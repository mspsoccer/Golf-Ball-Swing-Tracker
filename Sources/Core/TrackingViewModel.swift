//
//  TrackingViewModel.swift
//  GolfBallTracker
//
//  The conductor. Owns the pipeline stages, wires their callbacks together, and
//  publishes a minimal, render-ready state to SwiftUI. All @Published mutation
//  happens on the main actor; the heavy work stays off it.
//

import Foundation
import Combine
import simd

@MainActor
public final class TrackingViewModel: ObservableObject {

    @Published public private(set) var phase: TrackingPhase = .idle
    @Published public private(set) var baseline: CalibrationBaseline = .invalid

    /// Live tracked path in normalised view space (solid line).
    @Published public private(set) var trackedPoints: [CGPoint] = []
    /// Predicted continuation in normalised view space (dashed line).
    @Published public private(set) var predictedPoints: [CGPoint] = []

    /// Raw 3D states for the map overlay (meters, camera-anchored frame).
    @Published public private(set) var trackedStates: [BallState] = []
    /// Predicted continuation states for the map overlay.
    @Published public private(set) var predictedStates: [BallState] = []

    @Published public private(set) var launchConditions: LaunchConditions?
    @Published public private(set) var statusMessage: String = "Point at the ball to calibrate"

    // Injected as protocols, not concretes.
    private let calibrator: LiDARCalibrating
    private let camera: FrameProducing
    private let tracker: BallTracking
    private let predictor: TrajectoryPredicting
    private let renderer: TracerRendering?

    public init(calibrator: LiDARCalibrating,
                camera: FrameProducing,
                tracker: BallTracking,
                predictor: TrajectoryPredicting,
                renderer: TracerRendering? = nil) {
        self.calibrator = calibrator
        self.camera = camera
        self.tracker = tracker
        self.predictor = predictor
        self.renderer = renderer
        wire()
    }

    private func wire() {
        // 1) LiDAR locks the baseline -> arm the system.
        calibrator.onBaselineLocked = { [weak self] baseline in
            Task { @MainActor in self?.handleBaselineLocked(baseline) }
        }

        // 2) Camera hands kept frames straight to the tracker. Stays on the
        //    capture queue — hopping to main would serialise the hot path.
        //    Wire tracker.frameGate = cameraManager at the composition root.
        camera.onFrame = { [weak self] pixelBuffer, ts in
            self?.tracker.track(pixelBuffer: pixelBuffer, timestamp: ts)
        }

        // 3) Tracker's per-frame state -> live tracer (hop to main, coalesced).
        tracker.onStateUpdated = { [weak self] state in
            Task { @MainActor in self?.appendTrackedState(state) }
        }

        // 4) Apex -> shut the tracker down, hand off to the predictor.
        tracker.onApexReached = { [weak self] apexSamples in
            Task { @MainActor in self?.handleApex(apexSamples) }
        }
    }

    public func beginCalibration() {
        phase = .calibrating
        statusMessage = "Hold steady on the ball…"
        trackedPoints.removeAll(keepingCapacity: true)
        predictedPoints.removeAll(keepingCapacity: true)
        trackedStates.removeAll(keepingCapacity: true)
        predictedStates.removeAll(keepingCapacity: true)
        launchConditions = nil
        calibrator.startCalibration()
    }

    public func reset() {
        camera.stopCapture()
        calibrator.stopCalibration()
        tracker.reset()
        renderer?.clear()
        baseline = .invalid
        launchConditions = nil
        trackedPoints.removeAll(keepingCapacity: true)
        predictedPoints.removeAll(keepingCapacity: true)
        trackedStates.removeAll(keepingCapacity: true)
        predictedStates.removeAll(keepingCapacity: true)
        phase = .idle
        statusMessage = "Point at the ball to calibrate"
    }

    private func handleBaselineLocked(_ baseline: CalibrationBaseline) {
        guard baseline.isValid else {
            phase = .failed("LiDAR could not lock a baseline")
            statusMessage = "Calibration failed — try again"
            return
        }
        self.baseline = baseline

        // Free ARKit immediately; configure & start the high-FPS pipeline.
        calibrator.stopCalibration()
        tracker.configure(baseline: baseline)
        camera.startCapture()

        phase = .armed
        statusMessage = String(format: "Locked at %.2f m — take your swing", baseline.referenceDistance)
    }

    private func appendTrackedState(_ state: BallState) {
        if phase == .armed { phase = .tracking }   // first motion = launch
        let p = project(state)
        trackedPoints.append(p)
        trackedStates.append(state)
        renderer?.render(tracked: trackedPoints, predicted: predictedPoints)
    }

    private func handleApex(_ apexSamples: [BallState]) {
        // The tracker has already shut itself down by the time this fires.
        camera.stopCapture()
        phase = .predicting
        statusMessage = "Apex reached — predicting flight"

        // Run the (CPU-bound) physics off the main actor, publish back on it.
        Task.detached(priority: .userInitiated) { [predictor] in
            let predicted = predictor.predictTrajectory(from: apexSamples)
            let lc = predictor.lastLaunchConditions
            await MainActor.run { [weak self] in
                self?.finishPrediction(predicted, launch: lc)
            }
        }
    }

    private func finishPrediction(_ predicted: [BallState], launch: LaunchConditions?) {
        launchConditions = launch
        predictedPoints  = predicted.map(project(_:))
        predictedStates  = predicted
        renderer?.render(tracked: trackedPoints, predicted: predictedPoints)
        phase = .complete
        if let lc = launch {
            statusMessage = String(format: "Launch %.0f mph @ %.1f°",
                                   lc.launchSpeed * 2.23694,
                                   lc.launchAngle * 180 / .pi)
        } else {
            statusMessage = "Flight predicted"
        }
    }

    /// Pinhole projection of a world point into normalised 0...1 view
    /// coordinates (origin top-left), using the calibrated FOV.
    private func project(_ state: BallState) -> CGPoint {
        let p = state.position
        let z = max(p.z, 0.001)                       // avoid divide-by-zero
        let halfFOV = baseline.horizontalFOV * 0.5
        let aspect = baseline.frameSize.x / max(baseline.frameSize.y, 1)
        let focal = 1.0 / tan(halfFOV)                // normalised focal length

        let ndcX = (p.x / z) * focal                  // -1...1 horizontally
        let ndcY = (p.y / z) * focal * aspect         // -1...1 vertically

        let u = (ndcX * 0.5) + 0.5
        let v = 0.5 - (ndcY * 0.5)                    // flip: world +y is up
        return CGPoint(x: CGFloat(u), y: CGFloat(v))
    }
}
