//
//  Agent2Predictor.swift
//  GolfBallTracker
//
//  Physics Predictor. Pure math; no Vision or AVFoundation.
//
//  Pipeline:
//    1. fitLaunchConditions  — OLS linear regression (via Accelerate / vDSP) on
//       the pre-apex BallState array → stable initial velocity + backspin estimate.
//    2. integrateRK4         — 4th-order Runge-Kutta from the apex to y ≤ 0
//       accounting for gravity, aerodynamic drag (Cd), and Magnus lift (Cl).
//
//  Coordinate convention matches BallState.swift (right-handed, camera-anchored):
//    +x  target-right  +y  up  +z  depth (away from lens)
//  Backspin axis: –x  (ball top moves toward −z as seen from player's right)
//

import Foundation
import Accelerate   // vDSP_sve, vDSP_dotpr — OLS inner-product accumulation
import simd         // SIMD3<Float>, simd_cross, simd_length

public final class Agent2Predictor: TrajectoryPredicting {

    public private(set) var lastLaunchConditions: LaunchConditions?

    private static let g:      Float = 9.81                        // m/s²
    private static let rho:    Float = 1.225                       // kg/m³  sea-level
    private static let mass:   Float = 0.04593                     // kg     regulation ball
    private static let radius: Float = 0.02135                     // m
    private static let area:   Float = .pi * 0.02135 * 0.02135     // m²  πr²

    // Dimpled golf ball drag coefficient (smooth sphere ~0.47; dimples lower it to ~0.28).
    private static let Cd: Float = 0.28

    // kDrag = ½ρ·Cd·A / m  [units: 1/m]
    // Drag deceleration:  a_drag = −kDrag · |v| · v
    private static let kDrag: Float = 0.5 * rho * Cd * area / mass

    // kSpinEst = ρ·R·A / (4·m)  [dimensionless]
    // Derived from the small-spin-parameter Magnus approximation C_L ≈ 0.5·Sp:
    //   a_Magnus_y ≈ kSpinEst · ω · vz
    // Allows back-solving ω from the residual vertical acceleration.
    private static let kSpinEst: Float = (rho * radius * area) / (4 * mass)

    // RK4 time step and loop ceiling.
    private static let dt:       Float = 0.002    // 2 ms → ~3 mm positional error at 58 m/s
    private static let maxSteps: Int   = 20_000   // 40 s ceiling; typical shot < 8 s

    private static let kMinSamples = 5            // minimum pre-apex points for OLS

    public func predictTrajectory(from apexSamples: [BallState]) -> [BallState] {
        guard apexSamples.count >= Self.kMinSamples else { return [] }
        let lc = fitLaunchConditions(from: apexSamples)
        lastLaunchConditions = lc
        return integrateRK4(lc: lc)
    }

    private func fitLaunchConditions(from samples: [BallState]) -> LaunchConditions {
        // Normalise timestamps to the first sample so the OLS intercept is
        // the position at t = 0 (the start of the pre-apex window).
        let t0 = Float(samples[0].timestamp)
        let ts = samples.map { Float($0.timestamp) - t0 }

        // OLS linear fit:  p(t) = p₀ + v·t
        // The slope gives a noise-robust velocity estimate (more stable than
        // the finite-difference values already stored in BallState.velocity).
        let vx0 = linearSlope(t: ts, y: samples.map { $0.position.x })
        let vy0 = linearSlope(t: ts, y: samples.map { $0.position.y })
        let vz0 = linearSlope(t: ts, y: samples.map { $0.position.z })

        let v0    = SIMD3<Float>(vx0, vy0, vz0)
        let speed = simd_length(v0)
        // Launch angle: arctan(vertical / horizontal-plane speed)
        let angle = atan2(vy0, simd_length(SIMD2<Float>(vx0, vz0)))
        let spin  = estimateSpinRate(samples: samples, vz0: vz0)

        return LaunchConditions(
            initialVelocity: v0,
            launchSpeed:     speed,
            launchAngle:     angle,
            spinRate:        spin,
            origin:          samples[0].position
        )
    }

    /// Ordinary least-squares slope for a set of (t, y) sample pairs.
    ///
    ///   slope = (n·Σty − Σt·Σy) / (n·Σt² − (Σt)²)
    ///
    /// Uses vDSP_sve (sum) and vDSP_dotpr (inner product) from Accelerate so
    /// the four O(n) accumulations execute as a single SIMD pass each.
    private func linearSlope(t: [Float], y: [Float]) -> Float {
        let n  = t.count
        guard n >= 2 else { return 0 }
        let fn = Float(n)

        var sumT: Float  = 0
        var sumY: Float  = 0
        var sumT2: Float = 0
        var sumTY: Float = 0
        vDSP_sve(t,  1, &sumT,  vDSP_Length(n))
        vDSP_sve(y,  1, &sumY,  vDSP_Length(n))
        vDSP_dotpr(t, 1, t, 1, &sumT2, vDSP_Length(n))
        vDSP_dotpr(t, 1, y, 1, &sumTY, vDSP_Length(n))

        let denom = fn * sumT2 - sumT * sumT
        guard abs(denom) > 1e-9 else { return 0 }
        return (fn * sumTY - sumY * sumT) / denom
    }

    /// Estimates backspin ω (rad/s) from the Magnus residual in observed
    /// vertical acceleration: a_y = −g − kDrag·|v|·vy + kSpinEst·ω·vz.
    /// Solving for ω: mean(a_y_obs − (−g − kDrag·|v|·vy)) / (kSpinEst · mean(vz)).
    /// a_y_obs is a central-difference second derivative of position (more
    /// stable than differentiating the stored finite-difference velocities).
    private func estimateSpinRate(samples: [BallState], vz0: Float) -> Float {
        let n = samples.count
        // Need ≥ 3 points for central difference; need forward motion for Magnus.
        guard n >= 3, abs(vz0) > 1.0 else { return 524 }   // 524 rad/s ≈ 5 000 rpm

        var residualSum: Float = 0
        var vzSum:       Float = 0
        var count:       Float = 0

        for i in 1 ..< n - 1 {
            // dtFull = t[i+1] – t[i-1];  halfDt = Δt between adjacent samples.
            let dtFull = Float(samples[i+1].timestamp - samples[i-1].timestamp)
            guard dtFull > 1e-4 else { continue }
            let halfDt = dtFull * 0.5

            // Central-difference vertical acceleration from calibrated positions
            // (this cancels most of the per-frame detection noise).
            let ayObs = (samples[i+1].position.y
                       - 2 * samples[i].position.y
                       + samples[i-1].position.y) / (halfDt * halfDt)

            // Drag contribution to a_y.
            let v     = samples[i].velocity
            let speed = simd_length(v)
            let aDragY = -Self.kDrag * speed * v.y

            residualSum += ayObs - (-Self.g + aDragY)
            vzSum       += v.z
            count       += 1
        }

        guard count > 0, abs(vzSum) > 0 else { return 524 }

        let est = (residualSum / count) / (Self.kSpinEst * (vzSum / count))
        // Clamp to the physically plausible range for a golf ball:
        //   100 rad/s ≈   955 rpm  (low-spin driver off a slow swing)
        // 2 500 rad/s ≈ 23 900 rpm (high-spin wedge, absolute ceiling)
        return max(100, min(2_500, est))
    }

    /// Integrates the equations of motion from the apex (lc.origin, lc.initialVelocity)
    /// until the ball returns to ground level (y ≤ 0).
    ///
    /// RK4 on [pos, vel]: forces depend only on vel (not pos) because gravity is
    /// constant and drag/Magnus are velocity-dependent, so each RK4 stage only
    /// re-evaluates the acceleration at the stage's mid-point velocity.
    private func integrateRK4(lc: LaunchConditions) -> [BallState] {
        var pos      = lc.origin
        var vel      = lc.initialVelocity
        let spin     = lc.spinRate
        let dt       = Self.dt
        var t: Float = 0

        // Pre-allocate for a typical 6 s flight at 2 ms steps.
        var path = [BallState]()
        path.reserveCapacity(3_000)

        for _ in 0 ..< Self.maxSteps {
            // Stop when the ball reaches ground level.
            // The 0.05 s guard prevents an immediate exit if the origin is already at y ≈ 0.
            if pos.y <= 0 && t > 0.05 { break }

            let (dp1, dv1) = deriv(vel: vel,                spin: spin)
            let (dp2, dv2) = deriv(vel: vel + dv1*(dt*0.5), spin: spin)
            let (dp3, dv3) = deriv(vel: vel + dv2*(dt*0.5), spin: spin)
            let (dp4, dv4) = deriv(vel: vel + dv3*dt,       spin: spin)

            let sixth = dt / 6
            pos = pos + (dp1 + dp2*2 + dp3*2 + dp4) * sixth
            vel = vel + (dv1 + dv2*2 + dv3*2 + dv4) * sixth
            t  += dt

            path.append(BallState(position: pos,
                                  velocity: vel,
                                  timestamp: TimeInterval(t)))
        }

        return path
    }

    /// One RK4 derivative evaluation: (dpos/dt, dvel/dt) = (v, a_total).
    /// Forces: drag = −kDrag·|v|·v, gravity = (0, −g, 0), and Magnus lift
    /// = ½ρ·C_L(Sp)·A/m·|v|²·(ω̂ × v̂), where backspin axis ω̂ = (−1, 0, 0)
    /// so ω̂ × v̂ = (0, v̂_z, −v̂_y) and Sp = R·ω/|v|, C_L ≈ 0.5·Sp (capped at 0.20).
    @inline(__always)
    private func deriv(vel: SIMD3<Float>, spin: Float) -> (SIMD3<Float>, SIMD3<Float>) {
        let speed = simd_length(vel)
        guard speed > 0.1 else {
            // Below 0.1 m/s treat as effectively stationary — avoid division by zero.
            return (vel, SIMD3<Float>(0, -Self.g, 0))
        }

        // Aerodynamic drag (opposes velocity, magnitude scales as v²).
        let aDrag = vel * (-Self.kDrag * speed)

        // Variable-coefficient Magnus lift.
        let vHat    = vel / speed
        let sp      = Self.radius * spin / speed
        let cl      = min(0.5 * sp, 0.20)          // empirical cap
        let liftDir = SIMD3<Float>(0, vHat.z, -vHat.y)     // ω̂ × v̂
        let magnusScale = 0.5 * Self.rho * cl * Self.area / Self.mass * speed * speed
        let aMagnus = liftDir * magnusScale

        return (vel, aDrag + aMagnus + SIMD3<Float>(0, -Self.g, 0))
    }
}
