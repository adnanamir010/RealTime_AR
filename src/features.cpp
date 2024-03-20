#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

int main(int argc, char *argv[]) {
  // Open the default video camera
  cv::VideoCapture vidCap("./image/vid1.mp4");

  // Check if the video capture device has been successfully opened
  if (!vidCap.isOpened()) {
    std::cerr << "Error: Unable to open video device." << std::endl;
    return -2;
  }

  // Get the width and height of frames in the video stream
  double frameWidth = vidCap.get(cv::CAP_PROP_FRAME_WIDTH);
  double frameHeight = vidCap.get(cv::CAP_PROP_FRAME_HEIGHT);
  cv::Mat frame;
  std::string featureType = "orb"; // Initialize feature type as ORB

  while (true) {
    // Capture frame-by-frame
    vidCap >> frame;

    // If the frame is empty, break immediately
    if (frame.empty()) {
      std::cerr << "Error: Frame is empty." << std::endl;
      break;
    }

    // Convert the frame to grayscale
    cv::Mat grayFrame;
    cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

    if (featureType == "harris") {
      // Perform Harris corner detection
      cv::Mat harrisCorners, harrisCornersNorm;
      cv::cornerHarris(grayFrame, harrisCorners, 2, 3, 0.04);
      cv::normalize(harrisCorners, harrisCornersNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
      for (int i = 0; i < harrisCornersNorm.rows; i++) {
        for (int j = 0; j < harrisCornersNorm.cols; j++) {
          if ((int) harrisCornersNorm.at<float>(i, j) > 125) {
            cv::circle(frame, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 2);
          }
        }
      }
    } else if (featureType == "orb") {
      // Use ORB to find and draw keypoints
      cv::Ptr<cv::ORB> orbDetector = cv::ORB::create();
      std::vector<cv::KeyPoint> keypoints;
      orbDetector->detect(grayFrame, keypoints);
      cv::drawKeypoints(frame, keypoints, frame, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    }
    //     else if (featureType == "sift")
    // {
    //   // find the sift features
    //   cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    //   std::vector<cv::KeyPoint> keypoints;
    //   sift->detect(gray, keypoints);

    //   // draw the keypoints
    //   cv::drawKeypoints(frame, keypoints, frame, cv::Scalar(0, 255, 0));
    // }
    // else if (featureType == "surf")
    // {
    //   // find the surf features
    //   cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
    //   std::vector<cv::KeyPoint> keypoints;
    //   surf->detect(gray, keypoints);

    //   // draw the keypoints
    //   cv::drawKeypoints(frame, keypoints, frame, cv::Scalar(0, 0, 255));
    // }

    // Display the resulting frame
    cv::imshow("Frame", frame);

    // Press 'Q' on the keyboard to exit the loop
    char c = (char)cv::waitKey(25);
    if (c == 'q' || c == 27) break;
    else if (c == 'h') featureType = "harris";
    else if (c == 'o') featureType = "orb"; // Switch to ORB features with 'o' key
    else if (c == 's') featureType = "sift";
    else if (c == 'u') featureType = "surf";
  }

  // When everything is done, release the video capture object
  vidCap.release();
  cv::destroyAllWindows();

  return 0;
}