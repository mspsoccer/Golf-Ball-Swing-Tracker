//
//  AgentProtocols.swift
//  GolfBallTracker
//
//  Contracts that keep the pipeline stages decoupled. Nothing in here imports
//  AVFoundation, Vision or Accelerate — the protocols are the seam, the
//  concrete types live behind them. This lets us unit-test the physics
//  predictor with a synthetic apex array and no camera at all.
//

import Foundation
import CoreVideo

/// The high-level state machine the whole app moves through. Exactly one of
/// these is active at a time; transitions are owned by the ViewModel.
public enum TrackingPhase: Sendable, Equatable {
    case idle               // app open, nothing happening
    case calibrating        // ARKit/LiDAR measuring the resting ball
    case armed              // baseline locked, waiting for launch
    case tracking           // neural tracker live, ball in flight pre-apex
    case predicting         // apex reached, physics predictor integrating
    case complete           // full path available
    case failed(String)     // unrecoverable error with reason
}

public protocol LiDARCalibrating: AnyObject {
    /// Begin an ARKit session and start sampling the depth to the ball.
    func startCalibration()
    /// Tear down ARKit completely (called the instant the swing starts, to
    /// reclaim GPU/Neural-Engine budget for the neural tracker).
    func stopCalibration()
    /// Fired once a stable baseline has been measured.
    var onBaselineLocked: ((CalibrationBaseline) -> Void)? { get set }
}

public protocol FrameProducing: AnyObject {
    /// Configure and start the 240/120 FPS capture session.
    func startCapture()
    func stopCapture()

    /// Hot path. Called from the capture serial queue for every frame the
    /// pipeline decides to KEEP (dropped frames never reach here). The buffer
    /// is only valid for the duration of the call.
    var onFrame: ((CVPixelBuffer, TimeInterval) -> Void)? { get set }
}

public protocol BallTracking: AnyObject {
    /// Inject the locked baseline before tracking begins.
    func configure(baseline: CalibrationBaseline)

    /// Feed a kept frame. Implementations MUST be non-blocking from the
    /// caller's perspective: if a previous frame is still inferring, this
    /// frame should be dropped, not queued.
    func track(pixelBuffer: CVPixelBuffer, timestamp: TimeInterval)

    /// Emitted for every successfully tracked frame (drives the live tracer).
    var onStateUpdated: ((BallState) -> Void)? { get set }

    /// Emitted exactly once when v_y crosses zero. Carries the full pre-apex
    /// trajectory so the physics predictor can curve-fit launch conditions.
    /// After this fires the tracker shuts itself down.
    var onApexReached: (([BallState]) -> Void)? { get set }

    func reset()
}

/// Recovered launch conditions from curve-fitting the tracker's samples.
public struct LaunchConditions: Sendable, Equatable {
    public var initialVelocity: SIMD3<Float>   // m/s at the apex handoff point
    public var launchSpeed: Float              // |v| m/s
    public var launchAngle: Float              // radians above horizontal
    public var spinRate: Float                 // rad/s, estimated backspin
    public var origin: SIMD3<Float>            // position at handoff

    public init(initialVelocity: SIMD3<Float>,
                launchSpeed: Float,
                launchAngle: Float,
                spinRate: Float,
                origin: SIMD3<Float>) {
        self.initialVelocity = initialVelocity
        self.launchSpeed = launchSpeed
        self.launchAngle = launchAngle
        self.spinRate = spinRate
        self.origin = origin
    }
}

public protocol TrajectoryPredicting: AnyObject {
    /// Take the tracker's apex array, fit launch conditions, and run RK4 to
    /// the ground (y = 0). Returns the predicted continuation only (does not
    /// include the tracked portion).
    func predictTrajectory(from apexSamples: [BallState]) -> [BallState]

    /// Exposed for the UI / debugging.
    var lastLaunchConditions: LaunchConditions? { get }
}

public protocol TracerRendering: AnyObject {
    /// Draw the solid (tracked) and dashed (predicted) polylines. Called on
    /// the main thread. Points are in normalised 0...1 view space.
    func render(tracked: [CGPoint], predicted: [CGPoint])
    func clear()
}
