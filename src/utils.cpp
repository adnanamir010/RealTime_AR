
#include <iostream>
#include "../include/utils.h"
#include "opencv2/aruco.hpp"
#include <opencv2/opencv.hpp>

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

bool getArucoCorners(cv::Mat& src, std::vector<cv::Point2f>& corners) {
    // Clear previous data
    corners.resize(35,Point2f(0,0));

    // Marker detection variables
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary,parameters);

    // Detect markers
    detector.detectMarkers(src,markerCorners,markerIds,rejectedCandidates);

    for (int i = 0; i < markerIds.size(); i++) {
        int idx = markerIds[i];
        corners[idx] = markerCorners[i][0];
    }

    return markerCorners.size() == 35; // successfully extract Aruco corners
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