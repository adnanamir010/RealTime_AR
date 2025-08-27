# Real-Time Augmented Reality

This project is a C++ application that performs camera calibration and uses the results to create an augmented reality experience, overlaying virtual 3D objects onto a live video feed.

![Bunny](assets/cover2.gif)

<details>
<summary>⚠️DO NOT PRESS⚠️</summary>
YOU GOT RICKROLLED!!
<img src="assets/cover.gif" alt="Rickrolled">
</details>

## 📖 Project Description

This application brings virtual objects into the real world. By recognizing specific markers like a chessboard or an ArUco board in a video stream, it can accurately determine the camera's position and orientation. This allows it to render 3D objects that appear fixed to the markers in the physical environment, creating a seamless and interactive augmented reality display. The virtual objects will move and orient themselves realistically as the camera or the marker is moved.

## ✨ Features

Explore the core functionalities that power this augmented reality experience:

  * **Camera Calibration:**

      * Calculates the camera's intrinsic parameters and distortion coefficients using a standard chessboard pattern.
      * Saves the calibration data to a file, so you only need to calibrate once.

  * **Marker Detection:**

      * **Chessboard Detection:** Accurately finds the corners of a 9x6 chessboard.
      * **ArUco Board Detection:** Detects and identifies markers on a 5x7 ArUco board for robust tracking.

  * **Pose Estimation:**

      * Calculates the precise 3D position (translation) and orientation (rotation) of the detected marker relative to the camera in real-time.

  * **Augmented Reality Rendering:**

      * **Virtual 3D Objects:** Renders and overlays complex 3D models (OBJ files) onto the detected markers. Watch a virtual house, teddy bear, or other objects appear in your world\!
      * **3D Axis Projection:** Visualizes the orientation of the detected marker by drawing a 3D coordinate axis on it.
      * **Surface Projection:** Projects a 2D image onto the plane of the detected marker, making it look like the image is part of the surface.

  * **Robust Feature Tracking (Experimental):**

      * Includes functionality for detecting and tracking more general features in a scene, such as Harris corners and ORB features, opening the door for markerless AR.

## 🚀 Getting Started

To get a local copy up and running, follow these simple steps.

### Prerequisites

  * A C++ compiler
  * OpenCV with the `aruco` module
  * CMake

### Installation

1.  Clone the repository:
    ```sh
    git clone https://github.com/adnanamir010/realtime_ar.git
    ```
2.  Build the project using CMake.

## 💻 Usage

After building the project, you can run the application to start the camera feed.

1.  **Calibration:** First, point your camera at a 9x6 chessboard pattern from various angles and distances. Press the calibration key to calculate and save the camera parameters.
2.  **AR Mode:** Once calibrated, point the camera at a supported marker (chessboard or ArUco board). The application will automatically detect the marker and begin overlaying virtual objects. You can switch between different virtual objects and visualization modes using keyboard commands.