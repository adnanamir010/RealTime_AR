
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

bool getArucoCorners(Mat &src, vector<Point2f> &corners) {
    corners.resize(35, Point2f(0, 0));
    vector<int> markerIds;
    vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    Ptr<cv::aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
    Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    aruco::detectMarkers(src, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    for (int i = 0; i < markerIds.size(); i++) {
        int idx = markerIds[i];
        corners[idx] = markerCorners[i][0];
    }

    return markerCorners.size() == 35; // successfully extract Aruco corners
}