
#include "opencv2/aruco.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#ifndef PROJECT4_UTILS_H
#define PROJECT4_UTILS_H

vector<Vec3f> initWorldCoords (Size targetSize);
bool getChessboardCorners(Mat &src, vector<Point2f> &corners, Size targetSize);
bool getArucoCorners(Mat &src, vector<Point2f> &corners);

#endif //PROJECT4_UTILS_H
