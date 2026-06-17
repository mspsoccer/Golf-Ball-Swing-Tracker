//
//  ContentView.swift
//  GolfBallTracker
//
//  Split layout:
//    TOP half  — live camera feed + CAShapeLayer tracer overlay + HUD controls
//    BOTTOM half — TrajectoryMapView: 3D satellite terrain with trajectory polylines
//
//  Composition-root wiring example (@main / AppDelegate):
//
//      let tracer     = TracerRenderer()
//      let camera     = CameraManager()
//      let calibrator = LiDARCalibrator()
//      let agent1     = Agent1Tracker();  agent1.frameGate = camera
//      let agent2     = Agent2Predictor()
//      let vm = TrackingViewModel(calibrator: calibrator, camera: camera,
//                                 tracker: agent1, predictor: agent2, renderer: tracer)
//      ContentView(viewModel: vm, previewLayer: camera.makePreviewLayer(), tracer: tracer)
//

import SwiftUI
import AVFoundation

// MARK: - Live camera preview

/// UIView whose backing layer IS the AVCaptureVideoPreviewLayer — the same
/// technique the system Camera app uses, eliminating an extra compositing pass.
private final class CameraPreviewHostView: UIView {
    override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }

    var previewLayer: AVCaptureVideoPreviewLayer {
        layer as! AVCaptureVideoPreviewLayer   // guaranteed by layerClass
    }
}

/// UIViewRepresentable that grafts the session from CameraManager.makePreviewLayer()
/// onto the host view's own backing AVCaptureVideoPreviewLayer.
private struct CameraPreviewView: UIViewRepresentable {
    let previewLayer: AVCaptureVideoPreviewLayer

    func makeUIView(context: Context) -> CameraPreviewHostView {
        let host = CameraPreviewHostView()
        host.previewLayer.session      = previewLayer.session
        host.previewLayer.videoGravity = .resizeAspectFill
        if let conn = host.previewLayer.connection,
           conn.isVideoOrientationSupported {
            conn.videoOrientation = .landscapeRight
        }
        return host
    }

    func updateUIView(_ uiView: CameraPreviewHostView, context: Context) { }
}

// MARK: - Root view

public struct ContentView: View {

    @ObservedObject public var viewModel: TrackingViewModel

    private let previewLayer: AVCaptureVideoPreviewLayer
    private let tracer: TracerRenderer

    public init(viewModel: TrackingViewModel,
                previewLayer: AVCaptureVideoPreviewLayer,
                tracer: TracerRenderer) {
        self.viewModel    = viewModel
        self.previewLayer = previewLayer
        self.tracer       = tracer
    }

    public var body: some View {
        GeometryReader { geo in
            VStack(spacing: 0) {

                //TOP: camera + tracer + HUD 
                cameraPanel
                    .frame(width: geo.size.width, height: geo.size.height * 0.5)
                    .clipped()

                // Hairline separator so the two panels read as distinct.
                Rectangle()
                    .fill(Color(white: 0.15))
                    .frame(height: 1)

                // BOTTOM: 3D satellite map
                mapPanel
                    .frame(width: geo.size.width,
                           height: geo.size.height * 0.5 - 1)
            }
        }
        .background(Color.black)
        .ignoresSafeArea()
        .statusBarHidden(true)
        .persistentSystemOverlays(.hidden)
    }

    // MARK: - Camera panel (top half)

    private var cameraPanel: some View {
        ZStack(alignment: .top) {

            // Camera feed
            CameraPreviewView(previewLayer: previewLayer)
                .ignoresSafeArea(edges: .top)

            // CAShapeLayer tracer overlay
            TracerView(renderer: tracer)

            // HUD
            VStack(spacing: 0) {
                statusBanner
                    .padding(.top, 12)
                Spacer()
                controlRow
                    .padding(.bottom, 12)
            }
        }
    }

    @ViewBuilder
    private var mapPanel: some View {
        if #available(iOS 17, *) {
            TrajectoryMapView(
                trackedStates:   viewModel.trackedStates,
                predictedStates: viewModel.predictedStates
            )
        } else {
            Color.black
                .overlay(
                    Text("Satellite map requires iOS 17")
                        .font(.caption)
                        .foregroundStyle(.gray)
                )
        }
    }

    //Status banner

    private var statusBanner: some View {
        HStack(spacing: 8) {
            Image(systemName: phaseSymbol)
                .imageScale(.medium)
                .foregroundStyle(phaseAccent)
            Text(viewModel.statusMessage)
                .font(.system(.subheadline, design: .monospaced, weight: .semibold))
                .foregroundStyle(.white)
                .lineLimit(1)
                .minimumScaleFactor(0.8)
        }
        .padding(.horizontal, 18)
        .padding(.vertical, 10)
        .background(.ultraThinMaterial, in: Capsule())
        .shadow(color: .black.opacity(0.4), radius: 8, y: 2)
    }

    // Phase-adaptive controls

    @ViewBuilder
    private var controlRow: some View {
        HStack(spacing: 20) {
            switch viewModel.phase {

            case .idle, .failed:
                primaryButton(label: "Calibrate", icon: "scope", tint: .yellow) {
                    viewModel.beginCalibration()
                }

            case .calibrating:
                ProgressView()
                    .progressViewStyle(.circular)
                    .tint(.yellow)
                    .scaleEffect(1.2)
                ghostButton(label: "Cancel") { viewModel.reset() }

            case .armed:
                Text("Swing when ready")
                    .font(.system(.callout, design: .monospaced))
                    .foregroundStyle(.white.opacity(0.7))
                ghostButton(label: "Reset") { viewModel.reset() }

            case .tracking, .predicting:
                ghostButton(label: "Reset") { viewModel.reset() }

            case .complete:
                if let lc = viewModel.launchConditions { launchPill(lc) }
                primaryButton(label: "New Shot",
                              icon:  "arrow.counterclockwise",
                              tint:  .green) { viewModel.reset() }
            }
        }
    }

    // Reusable components
    private func primaryButton(label: String,
                                icon: String,
                                tint: Color,
                                action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Label(label, systemImage: icon)
                .font(.system(.headline, weight: .semibold))
                .padding(.horizontal, 24)
                .padding(.vertical, 12)
                .background(tint, in: Capsule())
                .foregroundStyle(.black)
        }
        .buttonStyle(.plain)
    }

    private func ghostButton(label: String, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Text(label)
                .font(.subheadline.weight(.medium))
                .padding(.horizontal, 18)
                .padding(.vertical, 9)
                .background(.ultraThinMaterial, in: Capsule())
                .foregroundStyle(.white)
        }
        .buttonStyle(.plain)
    }

    /// Compact telemetry card shown after a shot completes.
    private func launchPill(_ lc: LaunchConditions) -> some View {
        VStack(spacing: 3) {
            Text(String(format: "%.0f mph", lc.launchSpeed * 2.23694))
                .font(.system(.title3, design: .rounded, weight: .bold))
            // rad/s → rpm:  60 / (2π) ≈ 9.5493
            Text(String(format: "%.1f°  ·  %.0f rpm",
                        lc.launchAngle * 180 / .pi,
                        lc.spinRate * 9.5493))
                .font(.system(.caption, design: .monospaced))
        }
        .foregroundStyle(.white)
        .padding(.horizontal, 16)
        .padding(.vertical, 9)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
    }

    //Phase visual identity

    private var phaseSymbol: String {
        switch viewModel.phase {
        case .idle:        return "circle.dashed"
        case .calibrating: return "scope"
        case .armed:       return "checkmark.circle.fill"
        case .tracking:    return "play.circle.fill"
        case .predicting:  return "waveform.path.ecg"
        case .complete:    return "flag.checkered"
        case .failed:      return "exclamationmark.triangle.fill"
        }
    }

    private var phaseAccent: Color {
        switch viewModel.phase {
        case .idle:                return .gray
        case .calibrating, .armed: return .yellow
        case .tracking:            return .green
        case .predicting:          return .cyan
        case .complete:            return .mint
        case .failed:              return .red
        }
    }
}
