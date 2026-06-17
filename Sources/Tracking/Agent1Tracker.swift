//
//  Agent1Tracker.swift
//  GolfBallTracker
//
//  Neural Tracker.
//
//  Consumes CVPixelBuffer frames from the camera pipeline, runs a
//  YOLOv10-nano Core ML model via Vision, lifts each detected bounding-box
//  centroid into a calibrated 3D BallState, accumulates a trajectory, and fires
//  onApexReached the instant vertical velocity has been ≤ 0 for kApexConfirmN
//  consecutive frames after the ball was first observed rising.
//

import Foundation
import CoreML
import Vision
import CoreVideo
import simd
import os

/// The single method on CameraManager that re-opens the cyclic drop-frame gate.
/// The tracker calls it as its very last act for every frame (success or
/// drop) so the capture queue never stalls waiting for inference that
/// already finished.
public protocol FrameGateReleasing: AnyObject {
    func markFrameProcessed()
}

// CameraManager.markFrameProcessed() already exists; this just registers conformance.
extension CameraManager: FrameGateReleasing {}

public final class Agent1Tracker: NSObject, BallTracking {

    public var onStateUpdated: ((BallState) -> Void)?
    public var onApexReached:  (([BallState]) -> Void)?

    /// Injected by the composition root (TrackingViewModel) after init.
    /// Weak to avoid a retain cycle: CameraManager → tracker → CameraManager.
    public weak var frameGate: FrameGateReleasing?

    private static let kModelName      = "GolfBallDetector"   // YOLOv10-nano .mlmodelc
    private static let kMinConfidence: Float = 0.25
    private static let kApexConfirmN   = 3                    // consecutive vy ≤ 0 frames
    private static let kMaxTrajectory  = 2_000                // safety cap on array growth

    // Written once on workerQueue during loadModel(), read-only afterwards.
    private var vnRequest: VNCoreMLRequest?

    private let workerQueue = DispatchQueue(label: "golf.tracker.agent1",
                                            qos: .userInteractive)

    // stateLock guards all fields below. Writers: configure()/reset() (main
    // actor), processFrame() (workerQueue). Readers: processFrame() (workerQueue).
    private let stateLock     = NSLock()
    private var baseline:     CalibrationBaseline = .invalid
    private var trajectory:   [BallState] = []
    private var lastState:    BallState?
    private var vyHistory:    [Float] = []     // rolling window, max kApexConfirmN entries
    private var hasSeenRising = false          // true once vy > 0 is observed
    private var isShutdown    = false          // true after apex fires; gate remains open

    private let log = Logger(subsystem: "golf.tracker", category: "agent1")

    public override init() {
        super.init()
        loadModel()
    }

    // Called on the main actor.
    public func configure(baseline: CalibrationBaseline) {
        stateLock.lock()
        self.baseline = baseline
        stateLock.unlock()
        log.info("Agent 1 configured: refDist=\(baseline.referenceDistance, format: .fixed(precision: 3)) m, refRadiusPx=\(baseline.referenceRadiusPx, format: .fixed(precision: 1)) px")
    }

    // Called on the capture queue; must return immediately.
    public func track(pixelBuffer: CVPixelBuffer, timestamp: TimeInterval) {
        workerQueue.async { [weak self] in
            self?.processFrame(pixelBuffer, timestamp: timestamp)
        }
    }

    // Called on the main actor.
    public func reset() {
        stateLock.lock()
        baseline      = .invalid
        trajectory.removeAll(keepingCapacity: true)
        lastState     = nil
        vyHistory.removeAll(keepingCapacity: true)
        hasSeenRising = false
        isShutdown    = false
        stateLock.unlock()
        log.info("Agent 1 reset")
    }

    // Runs on workerQueue.
    private func processFrame(_ pixelBuffer: CVPixelBuffer, timestamp: TimeInterval) {
        // Gate is ALWAYS released here — even on the early-return paths below —
        // so the capture queue never gets stuck.
        defer { frameGate?.markFrameProcessed() }

        stateLock.lock()
        let shutdown = isShutdown
        stateLock.unlock()
        guard !shutdown else { return }

        guard let req = vnRequest else {
            // No compiled model in the bundle yet. Inject a synthetic centre
            // detection so the rest of the pipeline can be exercised in dev.
            handleDetection(normCentre: SIMD2<Float>(0.5, 0.5),
                            normBoxWidth: 0.04,
                            timestamp: timestamp)
            return
        }

        // Orientation: the connection was set to .landscapeRight so the buffer
        // is already in landscape. Pass .up so Vision makes no additional rotation.
        // Verify against the trained model's expected orientation if detections
        // appear mirrored or upside-down.
        let handler = VNImageRequestHandler(cvPixelBuffer: pixelBuffer,
                                            orientation: .up,
                                            options: [:])
        do {
            try handler.perform([req])
        } catch {
            log.error("Vision inference failed: \(error.localizedDescription)")
            return
        }

        guard let best = req.results?
            .compactMap({ $0 as? VNRecognizedObjectObservation })
            .filter({ ($0.labels.first?.confidence ?? 0) >= Self.kMinConfidence })
            .max(by: { ($0.labels.first?.confidence ?? 0) < ($1.labels.first?.confidence ?? 0) })
        else { return }

        let box = best.boundingBox   // Vision: origin bottom-left, x/y in 0…1
        handleDetection(normCentre:   SIMD2<Float>(Float(box.midX), Float(box.midY)),
                        normBoxWidth: Float(box.width),
                        timestamp:    timestamp)
    }

    // Runs on workerQueue. Converts a 2D detection into a 3D BallState.
    private func handleDetection(normCentre: SIMD2<Float>,
                                 normBoxWidth: Float,
                                 timestamp: TimeInterval) {
        stateLock.lock()
        let bl = baseline
        stateLock.unlock()
        guard bl.isValid, normBoxWidth > 0 else { return }

        // Depth from apparent size (pinhole invariant f*R = const):
        //   Z = referenceRadiusPx * referenceDistance / apparentRadiusPx
        let apparentRadiusPx = normBoxWidth * bl.frameSize.x * 0.5
        let Z = bl.referenceRadiusPx * bl.referenceDistance / max(apparentRadiusPx, 1.0)

        // Lateral/vertical position. Vision's y=0 is frame-bottom, which maps
        // to world −Y, so ndcY already has the right sign for +Y-up.
        let tanHalfFOV = tan(bl.horizontalFOV * 0.5)
        let aspect     = bl.frameSize.y / max(bl.frameSize.x, 1.0)
        let ndcX       = (normCentre.x - 0.5) * 2.0
        let ndcY       = (normCentre.y - 0.5) * 2.0

        let position = SIMD3<Float>(
            ndcX * Z * tanHalfFOV,
            ndcY * Z * tanHalfFOV * aspect,
            Z
        )

        // Velocity via finite difference.
        stateLock.lock()
        let prev = lastState
        stateLock.unlock()

        var velocity = SIMD3<Float>.zero
        if let prev, timestamp > prev.timestamp {
            let dt = Float(timestamp - prev.timestamp)
            velocity = (position - prev.position) / dt
        }

        let state = BallState(position: position, velocity: velocity, timestamp: timestamp)

        // Update trajectory and apex-detection window.
        stateLock.lock()
        if trajectory.count < Self.kMaxTrajectory { trajectory.append(state) }
        lastState = state

        let vy = velocity.y
        if vy > 0 { hasSeenRising = true }
        vyHistory.append(vy)
        if vyHistory.count > Self.kApexConfirmN { vyHistory.removeFirst() }

        // Snapshot everything we need for the apex check while still under lock.
        let rising    = hasSeenRising
        let vySnap    = vyHistory          // value-type copy
        let trajSnap  = trajectory         // value-type copy (see kMaxTrajectory cap above)
        stateLock.unlock()

        onStateUpdated?(state)
        checkApex(hasSeenRising: rising, vyHistory: vySnap, trajectory: trajSnap)
    }

    /// Fires onApexReached (and permanently shuts down the tracker) when:
    ///   1. The ball was observed rising (vy > 0) at least once.
    ///   2. The last kApexConfirmN velocity samples are all ≤ 0.
    private func checkApex(hasSeenRising: Bool,
                           vyHistory: [Float],
                           trajectory: [BallState]) {
        guard hasSeenRising,
              vyHistory.count == Self.kApexConfirmN,
              vyHistory.allSatisfy({ $0 <= 0 }) else { return }

        // Claim the shutdown under lock to guard against a concurrent reset().
        stateLock.lock()
        guard !isShutdown else { stateLock.unlock(); return }
        isShutdown = true
        stateLock.unlock()

        log.info("Apex confirmed — \(trajectory.count) tracked frames")
        onApexReached?(trajectory)
    }

    // Runs on workerQueue, async.
    private func loadModel() {
        workerQueue.async { [weak self] in
            guard let self else { return }

            // Accept either the compiled .mlmodelc or the source .mlpackage.
            guard let url = Bundle.main.url(forResource: Self.kModelName,
                                            withExtension: "mlmodelc")
                         ?? Bundle.main.url(forResource: Self.kModelName,
                                            withExtension: "mlpackage") else {
                self.log.warning("'\(Self.kModelName)' not found in bundle — synthetic detections active until model is added")
                return
            }

            do {
                let mlModel = try MLModel(contentsOf: url)
                let vnModel = try VNCoreMLModel(for: mlModel)
                let req = VNCoreMLRequest(model: vnModel)
                // scaleFill minimises letterboxing distortion for the bounding-box
                // pixel-to-angle mapping; the model was presumably trained this way.
                req.imageCropAndScaleOption = .scaleFill
                self.vnRequest = req
                self.log.info("Loaded Core ML model '\(Self.kModelName)'")
            } catch {
                self.log.error("Core ML model load failed: \(error.localizedDescription)")
            }
        }
    }
}
