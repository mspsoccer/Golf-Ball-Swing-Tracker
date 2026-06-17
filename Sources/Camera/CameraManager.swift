import Foundation
import AVFoundation
import ARKit
import CoreVideo
import simd
import os


public final class LiDARCalibrator: NSObject, LiDARCalibrating, ARSessionDelegate {

    public var onBaselineLocked: ((CalibrationBaseline) -> Void)?

    private let session = ARSession()
    private let sampleQueue = DispatchQueue(label: "golf.lidar.samples")

    // Rolling depth samples (meters). Pre-sized; never grows in steady state.
    private var depthSamples = [Float](repeating: 0, count: kSampleWindow)
    private var sampleCount = 0
    private var locked = false

    private static let kSampleWindow = 30   // ~0.5s at 60 Hz ARKit
    private let log = Logger(subsystem: "golf.tracker", category: "lidar")

    public func startCalibration() {
        guard ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) else {
            log.error("Device has no LiDAR / sceneDepth support")
            return
        }
        locked = false
        sampleCount = 0

        let config = ARWorldTrackingConfiguration()
        config.frameSemantics = .sceneDepth
        config.planeDetection = []                 // we only need depth
        session.delegate = self
        session.delegateQueue = sampleQueue
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        log.info("LiDAR calibration started")
    }

    public func stopCalibration() {
        session.pause()
        session.delegate = nil
        log.info("LiDAR calibration stopped (ARKit released)")
    }

    public func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard !locked, let sceneDepth = frame.sceneDepth else { return }

        let depth = centerDepth(from: sceneDepth.depthMap)
        guard depth > 0.2, depth < 10.0 else { return }   // reject noise

        depthSamples[sampleCount % Self.kSampleWindow] = depth
        sampleCount += 1

        guard sampleCount >= Self.kSampleWindow else { return }

        // Stable window collected -> lock a median baseline.
        locked = true
        let distance = median(of: depthSamples)
        let baseline = makeBaseline(distance: distance, frame: frame)
        log.info("Baseline locked at \(distance, format: .fixed(precision: 3)) m")
        onBaselineLocked?(baseline)
    }

    /// Reads the depth at the buffer centre. CVPixelBuffer of `kCVPixelFormatType_DepthFloat32`.
    private func centerDepth(from depthMap: CVPixelBuffer) -> Float {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        let w = CVPixelBufferGetWidth(depthMap)
        let h = CVPixelBufferGetHeight(depthMap)
        guard let base = CVPixelBufferGetBaseAddress(depthMap) else { return 0 }
        let rowBytes = CVPixelBufferGetBytesPerRow(depthMap)

        let cx = w / 2, cy = h / 2
        let row = base.advanced(by: cy * rowBytes)
        let ptr = row.assumingMemoryBound(to: Float32.self)
        return ptr[cx]
    }

    private func makeBaseline(distance: Float, frame: ARFrame) -> CalibrationBaseline {
        let intr = frame.camera.intrinsics
        let imgRes = frame.camera.imageResolution
        let fx = intr[0][0]
        let width = Float(imgRes.width)
        // Horizontal FOV from focal length: 2 * atan( (w/2) / fx ).
        let hFOV = 2 * atan((width * 0.5) / fx)

        // Apparent ball radius in pixels at this distance.
        // r_px = f * R_real / Z, standard pinhole. Golf ball R = 0.02135m
        let ballRadiusReal: Float = 0.02135
        let radiusPx = fx * ballRadiusReal / distance

        return CalibrationBaseline(
            referenceDistance: distance,
            referenceRadiusPx: radiusPx,
            horizontalFOV: hFOV,
            frameSize: SIMD2<Float>(Float(imgRes.width), Float(imgRes.height))
        )
    }

    private func median(of values: [Float]) -> Float {
        let sorted = values.sorted()
        return sorted[sorted.count / 2]
    }
}

public final class CameraManager: NSObject, FrameProducing,
                                  AVCaptureVideoDataOutputSampleBufferDelegate {

    public var onFrame: ((CVPixelBuffer, TimeInterval) -> Void)?

    // Serial queue that owns the session and receives sample buffers.
    private let captureQueue = DispatchQueue(label: "golf.camera.capture",
                                             qos: .userInteractive)
    private let session = AVCaptureSession()
    private let videoOutput = AVCaptureVideoDataOutput()
    private let log = Logger(subsystem: "golf.tracker", category: "camera")

    private let busy = ManagedAtomicFlag()

    private var configured = false
    private let targetFPS: Double = 240

    /// Creates a preview layer pre-wired to this session. Call once and embed
    /// the returned layer in the view hierarchy before calling startCapture().
    public func makePreviewLayer() -> AVCaptureVideoPreviewLayer {
        let layer = AVCaptureVideoPreviewLayer(session: session)
        layer.videoGravity = .resizeAspectFill
        if let conn = layer.connection, conn.isVideoOrientationSupported {
            conn.videoOrientation = .landscapeRight
        }
        return layer
    }

    public func startCapture() {
        captureQueue.async { [weak self] in
            guard let self else { return }
            if !self.configured { self.configureSession() }
            guard self.configured, !self.session.isRunning else { return }
            self.session.startRunning()
            self.log.info("Capture session running @ \(self.targetFPS, format: .fixed(precision: 0)) FPS target")
        }
    }

    public func stopCapture() {
        captureQueue.async { [weak self] in
            guard let self, self.session.isRunning else { return }
            self.session.stopRunning()
            self.busy.clear()
            self.log.info("Capture session stopped")
        }
    }

    public func markFrameProcessed() {
        busy.clear()
    }

    // Runs once, on captureQueue.
    private func configureSession() {
        session.beginConfiguration()
        session.sessionPreset = .inputPriority   // let the device/format decide

        guard let device = bestHighSpeedCamera() else {
            log.error("No suitable high-speed camera")
            session.commitConfiguration()
            return
        }

        do {
            guard let format = bestFormat(for: device, fps: targetFPS) else {
                log.error("No \(self.targetFPS, format: .fixed(precision: 0)) FPS format available")
                session.commitConfiguration()
                return
            }

            try device.lockForConfiguration()
            device.activeFormat = format
            let duration = CMTimeMake(value: 1, timescale: Int32(targetFPS))
            device.activeVideoMinFrameDuration = duration
            device.activeVideoMaxFrameDuration = duration
            // Fix focus/exposure: re-focusing mid-swing ruins scale estimation.
            if device.isFocusModeSupported(.locked) { device.focusMode = .locked }
            if device.isExposureModeSupported(.locked) { device.exposureMode = .locked }
            device.unlockForConfiguration()

            let input = try AVCaptureDeviceInput(device: device)
            if session.canAddInput(input) { session.addInput(input) }
        } catch {
            log.error("Camera config failed: \(error.localizedDescription)")
            session.commitConfiguration()
            return
        }

        // discard late frames so OS never queues backlog behind us.
        videoOutput.videoSettings = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA
        ]
        videoOutput.alwaysDiscardsLateVideoFrames = true
        videoOutput.setSampleBufferDelegate(self, queue: captureQueue)
        if session.canAddOutput(videoOutput) { session.addOutput(videoOutput) }

        if let conn = videoOutput.connection(with: .video),
           conn.isVideoOrientationSupported {
            conn.videoOrientation = .landscapeRight  
        }

        session.commitConfiguration()
        configured = true
    }

    private func bestHighSpeedCamera() -> AVCaptureDevice? {
        let discovery = AVCaptureDevice.DiscoverySession(
            deviceTypes: [.builtInWideAngleCamera],
            mediaType: .video,
            position: .back)
        return discovery.devices.first
    }

    private func bestFormat(for device: AVCaptureDevice, fps: Double) -> AVCaptureDevice.Format? {
        var best: AVCaptureDevice.Format?
        var bestPixels = 0
        for format in device.formats {
            let supports = format.videoSupportedFrameRateRanges.contains {
                $0.maxFrameRate >= fps
            }
            guard supports else { continue }
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            let pixels = Int(dims.width) * Int(dims.height)
            if pixels > bestPixels { bestPixels = pixels; best = format }
        }
        return best
    }


    public func captureOutput(_ output: AVCaptureOutput,
                              didOutput sampleBuffer: CMSampleBuffer,
                              from connection: AVCaptureConnection) {
        // Cyclic drop-frame gate: if the tracker hasn't released the previous
        // frame, drop this one outright. compareExchange is lock-free.
        guard busy.testAndSet() == false else { return }  // was busy -> drop

        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else {
            busy.clear()
            return
        }
        let pts = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)
        let ts = CMTimeGetSeconds(pts)

        onFrame?(pixelBuffer, ts)
    }
    public func captureOutput(_ output: AVCaptureOutput,
                              didDrop sampleBuffer: CMSampleBuffer,
                              from connection: AVCaptureConnection) {
        // OS-level drop (queue full). Expected at 240 FPS
    }
}


public final class ManagedAtomicFlag {
    private var lock = os_unfair_lock_s()
    private var busy = false

    @inline(__always)
    public func testAndSet() -> Bool {
        os_unfair_lock_lock(&lock)
        let previous = busy
        busy = true
        os_unfair_lock_unlock(&lock)
        return previous
    }

    @inline(__always)
    public func clear() {
        os_unfair_lock_lock(&lock)
        busy = false
        os_unfair_lock_unlock(&lock)
    }
}
