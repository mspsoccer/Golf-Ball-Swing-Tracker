//
//  TrajectoryMapView.swift
//  GolfBallTracker
//  Coordinate conversion
//  BallState positions are in meters relative to the camera (right-hand frame):
//    +x  target-right (lateral)      +y  up (unused for geo)      +z  depth (forward)
//
//  Given the device's GPS fix and compass true-heading θ, the geographic offset is:
//    north = z·cosθ − x·sinθ   (meters)
//    east  = z·sinθ + x·cosθ   (meters)
//    Δlat  = north / 111 111
//    Δlon  = east  / (111 111 · cos(lat₀))
//
//  SETUP NOTE
//  Add NSLocationWhenInUseUsageDescription to Info.plist, e.g.:
//    "GolfBallTracker uses your location to overlay trajectories on the course map."
//  Without it the system silently denies the request and the fallback coordinate is used.
//
//  Requires iOS 17 (MapPolyline, MapCamera, Map(position:), .imagery(elevation:)).
//

import SwiftUI
import MapKit
import CoreLocation

// MARK: - Location + compass provider

/// Wraps CLLocationManager to vend GPS fix and true-heading as @Published values.
final class ShotLocationTracker: NSObject, ObservableObject, CLLocationManagerDelegate {

    @Published var coordinate: CLLocationCoordinate2D?
    @Published var trueHeading: CLLocationDegrees = 0   // degrees from north, clockwise

    private let manager = CLLocationManager()

    override init() {
        super.init()
        manager.delegate          = self
        manager.desiredAccuracy   = kCLLocationAccuracyBestForNavigation
        manager.headingFilter     = 1   // update every 1° of change
        manager.requestWhenInUseAuthorization()
        manager.startUpdatingLocation()
        manager.startUpdatingHeading()
    }

    func locationManager(_ manager: CLLocationManager,
                         didUpdateLocations locations: [CLLocation]) {
        coordinate = locations.last?.coordinate
    }

    func locationManager(_ manager: CLLocationManager,
                         didUpdateHeading newHeading: CLHeading) {
        guard newHeading.headingAccuracy >= 0 else { return }
        trueHeading = newHeading.trueHeading
    }

    func locationManagerShouldDisplayHeadingCalibration(
        _ manager: CLLocationManager) -> Bool { true }
}

// MARK: - Coordinate conversion (file-private)

/// Projects a BallState's local-frame (x, z) position onto the Earth's surface
/// using the camera's GPS origin and compass heading.
private func geoCoord(for state: BallState,
                      origin: CLLocationCoordinate2D,
                      headingDeg: Double) -> CLLocationCoordinate2D {
    let θ  = headingDeg * .pi / 180        // heading in radians
    let x  = Double(state.position.x)      // meters right of camera
    let z  = Double(state.position.z)      // meters in front of camera

    // Decompose into north/east displacement (standard rotation by heading).
    let north = z * cos(θ) - x * sin(θ)
    let east  = z * sin(θ) + x * cos(θ)

    let metersPerDegLat = 111_111.0
    let metersPerDegLon = 111_111.0 * cos(origin.latitude * .pi / 180)

    return CLLocationCoordinate2D(
        latitude:  origin.latitude  + north / metersPerDegLat,
        longitude: origin.longitude + east  / metersPerDegLon
    )
}

// MARK: - Map view

/// Satellite terrain map showing the ball's tracked and predicted paths.
/// Requires iOS 17 for MapPolyline, MapCamera, and .imagery(elevation: .realistic).
@available(iOS 17, *)
public struct TrajectoryMapView: View {

    public let trackedStates:   [BallState]
    public let predictedStates: [BallState]

    @StateObject private var location = ShotLocationTracker()
    @State private var cameraPosition: MapCameraPosition = .automatic

    /// University of Maryland Golf Course — active when device location is unavailable.
    private static let fallbackOrigin = CLLocationCoordinate2D(
        latitude: 38.9897, longitude: -76.9378
    )

    public var body: some View {
        Map(position: $cameraPosition) {
            let origin  = currentOrigin()
            let heading = location.trueHeading

            // ── Tracked path (solid yellow, 4 pt) ───────────────────────────
            let trackedCoords = trackedStates.map {
                geoCoord(for: $0, origin: origin, headingDeg: heading)
            }
            if trackedCoords.count >= 2 {
                MapPolyline(coordinates: trackedCoords)
                    .stroke(.yellow, lineWidth: 4)
            }

            // ── Predicted continuation (dashed cyan, 3 pt) ───────────────────
            let predictedCoords = predictedStates.map {
                geoCoord(for: $0, origin: origin, headingDeg: heading)
            }
            if predictedCoords.count >= 2 {
                MapPolyline(coordinates: predictedCoords)
                    .stroke(
                        Color.cyan.opacity(0.85),
                        style: StrokeStyle(lineWidth: 3, dash: [12, 7])
                    )
            }

            // ── Tee / camera origin marker ───────────────────────────────────
            Annotation("Tee", coordinate: origin, anchor: .bottom) {
                Image(systemName: "camera.fill")
                    .font(.system(size: 10, weight: .bold))
                    .foregroundStyle(.white)
                    .padding(6)
                    .background(.black.opacity(0.75), in: Circle())
            }

            // ── Predicted landing spot ───────────────────────────────────────
            if let landingCoord = predictedCoords.last {
                Annotation("Landing", coordinate: landingCoord, anchor: .bottom) {
                    Image(systemName: "flag.fill")
                        .font(.system(size: 10, weight: .bold))
                        .foregroundStyle(.white)
                        .padding(6)
                        .background(.green.opacity(0.9), in: Circle())
                }
            }
        }
        .mapStyle(.imagery(elevation: .realistic))
        .mapControls { }    // suppress default controls — space is limited in split layout
        .onAppear      { recenterCamera() }
        .onChange(of: trackedStates)               { recenterCamera() }
        .onChange(of: predictedStates)             { recenterCamera() }
        .onChange(of: location.coordinate?.latitude) { recenterCamera() }
    }

    // MARK: - Camera management

    private func currentOrigin() -> CLLocationCoordinate2D {
        location.coordinate ?? Self.fallbackOrigin
    }

    /// Recomputes the camera so it frames all trajectory points with a 45° down-look.
    /// When no trajectory exists the camera rests on the tee at 300 m altitude.
    private func recenterCamera() {
        let origin   = currentOrigin()
        let heading  = location.trueHeading
        let allCoords = (trackedStates + predictedStates).map {
            geoCoord(for: $0, origin: origin, headingDeg: heading)
        }

        guard !allCoords.isEmpty else {
            cameraPosition = .camera(MapCamera(
                centerCoordinate: origin,
                distance: 300,
                heading: heading,
                pitch:   45
            ))
            return
        }

        // Centroid of the trajectory.
        let n = Double(allCoords.count)
        let centroid = CLLocationCoordinate2D(
            latitude:  allCoords.map(\.latitude).reduce(0, +) / n,
            longitude: allCoords.map(\.longitude).reduce(0, +) / n
        )

        // Pull back so the farthest point stays inside the frame with margin.
        let cosLat = cos(centroid.latitude * .pi / 180)
        let maxDist = allCoords.map { coord -> Double in
            let dlat = (coord.latitude  - centroid.latitude)  * 111_111
            let dlon = (coord.longitude - centroid.longitude) * 111_111 * cosLat
            return sqrt(dlat*dlat + dlon*dlon)
        }.max() ?? 150

        cameraPosition = .camera(MapCamera(
            centerCoordinate: centroid,
            distance: max(200, maxDist * 2.8),   // 2.8× gives comfortable margins
            heading: heading,
            pitch:   45   // angled top-down: shows 3D terrain while reading footprint
        ))
    }
}
