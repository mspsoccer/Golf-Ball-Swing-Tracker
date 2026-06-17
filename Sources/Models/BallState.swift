//
//  BallState.swift
//  GolfBallTracker
//
//  Value types shared by the camera pipeline, the neural tracker, and the
//  physics predictor. All fields are SI units (meters, meters/second,
//  seconds) in a right-handed, camera-anchored frame: +x target-right,
//  +y up, +z depth away from the lens. Origin is the ball's resting
//  position at calibration.
//

import Foundation
import simd

/// Immutable snapshot of the ball at one instant in time.
public struct BallState: Sendable, Equatable {

    /// Position in meters.
    public var position: SIMD3<Float>

    /// Velocity in meters/second.
    public var velocity: SIMD3<Float>

    /// Capture timestamp in seconds (monotonic, not wall-clock).
    public var timestamp: TimeInterval

    @inline(__always) public var x: Float { position.x }
    @inline(__always) public var y: Float { position.y }
    @inline(__always) public var z: Float { position.z }

    /// Vertical velocity (m/s); crosses zero at the swing's apex.
    @inline(__always) public var vy: Float { velocity.y }

    /// Scalar speed (m/s).
    @inline(__always) public var speed: Float { simd_length(velocity) }

    public init(position: SIMD3<Float>,
                velocity: SIMD3<Float>,
                timestamp: TimeInterval) {
        self.position = position
        self.velocity = velocity
        self.timestamp = timestamp
    }

    /// Resting state used to pre-fill ring buffers.
    public static let zero = BallState(position: .zero,
                                       velocity: .zero,
                                       timestamp: 0)
}

/// Raw detection from the tracker's model, before 3D projection.
public struct BallDetection: Sendable, Equatable {
    /// Normalized bounding box in Vision coordinates (origin bottom-left).
    public var boundingBox: CGRect
    /// Model confidence, 0...1.
    public var confidence: Float
    public var timestamp: TimeInterval

    public init(boundingBox: CGRect, confidence: Float, timestamp: TimeInterval) {
        self.boundingBox = boundingBox
        self.confidence = confidence
        self.timestamp = timestamp
    }
}

/// Output of the LiDAR calibrator, captured once before the swing.
public struct CalibrationBaseline: Sendable, Equatable {
    /// Camera-to-ball distance at rest, in meters.
    public var referenceDistance: Float
    /// Ball's apparent radius in pixels at `referenceDistance`.
    public var referenceRadiusPx: Float
    /// Camera horizontal field of view, in radians.
    public var horizontalFOV: Float
    /// Pixel dimensions of the tracked frames.
    public var frameSize: SIMD2<Float>

    public init(referenceDistance: Float,
                referenceRadiusPx: Float,
                horizontalFOV: Float,
                frameSize: SIMD2<Float>) {
        self.referenceDistance = referenceDistance
        self.referenceRadiusPx = referenceRadiusPx
        self.horizontalFOV = horizontalFOV
        self.frameSize = frameSize
    }

    public static let invalid = CalibrationBaseline(referenceDistance: 0,
                                                    referenceRadiusPx: 0,
                                                    horizontalFOV: 0,
                                                    frameSize: .zero)

    public var isValid: Bool { referenceDistance > 0 && referenceRadiusPx > 0 }
}
