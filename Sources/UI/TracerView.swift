//
//  TracerView.swift
//  GolfBallTracker
//
//  Render Engine.
//
//  TracerRenderer owns two CAShapeLayers (one solid, one dashed) and conforms
//  to TracerRendering. It receives normalised 0…1 CGPoints from TrackingViewModel
//  and maps them to physical pixels in the canvas's current bounds.
//
//  TracerView is a zero-config UIViewRepresentable that places the canvas into
//  the SwiftUI hierarchy. Use .ignoresSafeArea() at the call site.
//
//  Thread safety: render() and clear() are always called on the main actor
//  (both entry points — appendTrackedState and finishPrediction — are @MainActor),
//  so no additional synchronisation is needed here.
//

import UIKit
import SwiftUI

/// UIView subclass that stretches its sublayers to fill its bounds whenever
/// Auto Layout assigns a new frame — handles rotation and split-screen resizing.
final class TracerCanvas: UIView {
    override func layoutSubviews() {
        super.layoutSubviews()
        layer.sublayers?.forEach { $0.frame = bounds }
    }
}

/// The concrete TracerRendering implementation the ViewModel talks to.
public final class TracerRenderer: TracerRendering {

    /// Embed this view in the SwiftUI hierarchy via TracerView.
    let canvas = TracerCanvas()

    // Solid yellow — live tracker detections.
    private let trackedLayer   = CAShapeLayer()
    // Dashed cyan — predicted RK4 continuation.
    private let predictedLayer = CAShapeLayer()

    public init() {
        styleTracked()
        stylePredicted()
        canvas.backgroundColor = .clear
        canvas.isUserInteractionEnabled = false
        canvas.layer.addSublayer(trackedLayer)
        canvas.layer.addSublayer(predictedLayer)
    }

    public func render(tracked: [CGPoint], predicted: [CGPoint]) {
        let bounds = canvas.bounds
        // Disable implicit CALayer animations. Without this, every path swap
        // triggers a 0.25 s cross-dissolve — unusable at 60 Hz.
        CATransaction.begin()
        CATransaction.setDisableActions(true)
        trackedLayer.path   = polyline(from: tracked,   in: bounds)
        predictedLayer.path = polyline(from: predicted, in: bounds)
        CATransaction.commit()
    }

    public func clear() {
        CATransaction.begin()
        CATransaction.setDisableActions(true)
        trackedLayer.path   = nil
        predictedLayer.path = nil
        CATransaction.commit()
    }

    private func styleTracked() {
        trackedLayer.strokeColor  = UIColor.systemYellow.cgColor
        trackedLayer.fillColor    = UIColor.clear.cgColor
        trackedLayer.lineWidth    = 3.5
        trackedLayer.lineCap      = .round
        trackedLayer.lineJoin     = .round
        // Soft halo so the line reads against both bright sky and dark fairway.
        trackedLayer.shadowColor   = UIColor.systemYellow.cgColor
        trackedLayer.shadowOpacity = 0.65
        trackedLayer.shadowRadius  = 7
        trackedLayer.shadowOffset  = .zero
    }

    private func stylePredicted() {
        predictedLayer.strokeColor    = UIColor.systemCyan.cgColor
        predictedLayer.fillColor      = UIColor.clear.cgColor
        predictedLayer.lineWidth      = 2.5
        predictedLayer.lineDashPattern = [10, 6]   // dash 10 pt, gap 6 pt
        predictedLayer.lineCap        = .round
        predictedLayer.lineJoin       = .round
        predictedLayer.shadowColor    = UIColor.systemCyan.cgColor
        predictedLayer.shadowOpacity  = 0.5
        predictedLayer.shadowRadius   = 5
        predictedLayer.shadowOffset   = .zero
    }

    /// Converts normalised 0…1 points to view-local pixels and builds a polyline.
    private func polyline(from points: [CGPoint], in bounds: CGRect) -> CGPath? {
        guard points.count >= 2 else { return nil }
        let path = UIBezierPath()
        path.move(to: denormalise(points[0], in: bounds))
        for pt in points.dropFirst() {
            path.addLine(to: denormalise(pt, in: bounds))
        }
        return path.cgPath
    }

    @inline(__always)
    private func denormalise(_ p: CGPoint, in bounds: CGRect) -> CGPoint {
        CGPoint(x: p.x * bounds.width, y: p.y * bounds.height)
    }
}

/// Drop-in SwiftUI view — just hosts the TracerRenderer's canvas.
/// Pair with `.ignoresSafeArea()` so the overlay reaches the screen edges.
struct TracerView: UIViewRepresentable {
    let renderer: TracerRenderer

    func makeUIView(context: Context) -> UIView { renderer.canvas }
    func updateUIView(_ uiView: UIView, context: Context) { }
}
