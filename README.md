# AI-Powered Golf Trajectory Tracker (iOS)

> **Development Status:** Active development in progress. Core computer vision tracking logic and location services are currently being implemented and tested.

## Overview
An iOS application built to autonomously track a golf ball's flight path directly from mobile camera footage. The system processes video frames to identify the ball's trajectory and uses native device sensors to map the flight path onto a coordinate system.

## Tech Stack
* Swift
* SwiftUI
* MapKit
* CoreLocation

## Current Architecture & Features
* Using device GPS and true-heading compass tracking to make precise geographic offsets for ball flight relative to the camera position.
* Building out the trajectory map view to overlay the tracked shot onto a satellite course map.
* Creating a low-latency processing pipeline to handle the frame-by-frame analysis needed for high-speed sports tracking.

## Local Setup

1. Clone the repository:
   `git clone https://github.com/mspsoccer/ai-swing-tracker.git`
2. Open the project directory in Xcode.
3. Select a target device. 
   *Note: Requires iOS 17+ to support the latest MapPolyline and MapCamera features. A physical device is recommended to test the CoreLocation heading/GPS features.*
4. Build and run the application.

## Author
Milan Patel
Software Engineer
