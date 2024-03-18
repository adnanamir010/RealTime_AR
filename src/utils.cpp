
#include <iostream>
#include "../include/utils.h"
#include "opencv2/aruco.hpp"
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using namespace cv;

vector<Vec3f> initWorldCoords (Size targetSize){
    vector<Vec3f> points;
    for (int i = 0; i < targetSize.height; i++) {
        for (int j = 0; j < targetSize.width; j++) {
            Vec3f coordinates = Vec3f(j, -i, 0);
            points.push_back(coordinates);
        }
    }
    return points;
}

bool getChessboardCorners(Mat &src, vector<Point2f> &corners, Size targetSize) {
    bool foundCorners = findChessboardCorners(src, targetSize, corners);
    if (foundCorners) {
        Mat gray;
        cvtColor(src, gray, COLOR_BGR2GRAY); // the input image for cornerSubPix must be single-channel
        Size subPixWinSize(10, 10);
        TermCriteria termCrit(TermCriteria::COUNT|TermCriteria::EPS, 1, 0.1);
        cornerSubPix(gray, corners, subPixWinSize, Size(-1, -1), termCrit);
    }
    return foundCorners;
}

bool getArucoCorners(cv::Mat& src, std::vector<cv::Point2f>& corners,vector<Point2f> &outer4) {

    corners.resize(35,Point2f(0,0));
    outer4.resize(4, Point2f(0, 0));

    // Marker detection variables
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners,rejectedCandidates;
    cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary,parameters);

    // Detect markers
    detector.detectMarkers(src,markerCorners,markerIds,rejectedCandidates);

    for (int i = 0; i < markerIds.size(); i++) {
        int idx = markerIds[i];
        corners[idx] = markerCorners[i][0];
    }

    for (int i = 0; i < markerIds.size(); i++) {
        int idx = markerIds[i];
        if (idx == 30) {
            outer4[0] = markerCorners[i][3];
        } else if (idx == 0) {
            outer4[1] = markerCorners[i][0];
        } else if (idx == 34) {
            outer4[2] = markerCorners[i][2];
        } else if (idx == 4) {
            outer4[3] = markerCorners[i][1];
        }
    }


    return markerCorners.size() == 35; // successfully extract Aruco corners
}

// Function to overlay an image onto a chessboard given the chessboard corners.
void overlayImage(Mat &src, Mat &dst, vector<Point2f> &corners) {
    // extract top-left, top-right, bottom-right, bottom-left for source image corners
    vector<Point2f> srcCorners = { Point2f(0, 0), Point2f(src.size().width, 0),
                                   Point2f(0, src.size().height),Point2f(src.size().width, src.size().height)};

    // Compute the homography matrix
    Mat homography = findHomography(srcCorners, corners);

    // Warp the source image to the destination based on the homography
    Mat warpedSrc;
    warpPerspective(src, warpedSrc, homography, dst.size());
    for (int i = 0; i < warpedSrc.rows; i++) {
        for (int j = 0; j < warpedSrc.cols; j++) {
            if (warpedSrc.at<Vec3b>(i, j) != Vec3b(0, 0, 0)) {
                dst.at<Vec3b>(i, j) = warpedSrc.at<Vec3b>(i, j);
            }
        }
    }

}

void mprint(cv::Mat x){
    for (int i=0;i<x.rows;i++){
        for (int j=0; j<x.cols;j++){
            cout << x.at<double>(i,j) << ",";
        }
        cout << endl;
    }
}

void rad2deg(cv::Mat x){
    for (int i=0;i<x.rows;i++){
        for (int j=0; j<x.cols;j++){
            x.at<double>(i,j)*=180/M_PI;
        }
    }
}


//bool getArucoCorners(cv::Mat& src, std::vector<cv::Point2f>& corners) {
//    // Clear previous data
//    corners.clear();
//
//    // Marker detection variables
//    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//    std::vector<int> markerIds;
//    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
//    cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();
//    cv::aruco::ArucoDetector detector(dictionary,parameters);
//
//    // Detect markers
//    detector.detectMarkers(src,markerCorners,markerIds,rejectedCandidates);
//
//    // Check if exactly 35 markers are detected
//    if(markerIds.size() == 35) {
//        // Extract top-left corners of the detected markers
//        for(const auto& cornerSet : markerCorners) {
//            if(!cornerSet.empty()) {
//                corners.push_back(cornerSet[0]); //top-left corner
//            }
//        }
//        return true; // Indicate that exactly 35 markers were detected
//    } else {
//        return false; // Indicate that the number of detected markers is not 35
//    }
//}