/*
 * Adnan Amir and Srijan Dokania
 * CS 5330 PRCV
 * Spring 2024
 * Purpose of file: Header file for various utility functions
 */

#include "opencv2/aruco.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#ifndef PROJECT4_UTILS_H
#define PROJECT4_UTILS_H

/**
 * Initializes a grid of world coordinates in a plane.
 * @param targetSize The size of the grid, specified as a number of columns (width) and rows (height).
 * @return A vector of Vec3f, each representing the (x, y, z) coordinates of a point in the grid.
 */
vector<Vec3f> initWorldCoords (Size targetSize);

/**
 * Detects chessboard corners in an image.
 * @param src The source image in which to find chessboard corners.
 * @param corners A reference to a vector where detected corner points will be stored.
 * @param targetSize The dimensions of the chessboard pattern (number of inner corners per chessboard row and column).
 * @return True if the specified number of corners are found, false otherwise.
 */
bool getChessboardCorners(Mat &src, vector<Point2f> &corners, Size targetSize);

/**
 * Detects corners of Aruco markers and the outer corners of a specific set of markers in an image.
 * @param src The source image in which to find Aruco markers.
 * @param corners A reference to a vector where detected marker corner points will be stored.
 * @param outer4 A reference to a vector where the outer 4 corners of the Arucoboard will be stored.
 * @return True if all expected Aruco markers are found, false otherwise.
 */
bool getArucoCorners(Mat &src, vector<Point2f> &corners,vector<Point2f> &outer4);

/**
 * Overlays an image onto a specified region in another image.
 * @param src The source image to overlay.
 * @param dst The destination image where the source image will be overlaid.
 * @param corners The corners of the destination region where the source image will be overlaid.
 */
void overlayImage(Mat &src, Mat &dst, vector<Point2f> &corners);

/**
 * Prints the elements of a matrix.
 * @param x The cv::Mat matrix to print.
 */
void mprint(cv::Mat x);

/**
 * Converts radians to degrees in a matrix.
 * @param x The cv::Mat matrix containing radian values to convert to degrees.
 */
void rad2deg(cv::Mat x);

/**
 * Parses the OBJ file specified by fName and populates vectors for vertices, normals, and face vertices.
 * Vertices are stored as cv::Point3f objects representing 3D points.
 * Normals are also stored as cv::Point3f objects representing the normal vectors.
 * Face vertices are stored as vectors of integers, where each integer represents an index to the vertices vector.
 * @param fName The name (and path) of the .obj file to parse.
 * @param vertices A reference to a vector of cv::Point3f where parsed vertices will be stored.
 * @param normals A reference to a vector of cv::Point3f where parsed normal vectors will be stored.
 * @param faceVertices A reference to a vector of vectors of integers where parsed face vertices indices will be stored.
 */
void parseOBJFile(const std::string &fName, std::vector<cv::Point3f> &vertices, std::vector<cv::Point3f> &normals, std::vector<std::vector<int>> &faceVertices);

int overlayCorners(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Vec3d rvec, cv::Vec3d tvec, cv::Mat &frame);

int overlayObject(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Vec3d& rvec, cv::Vec3d& tvec, std::vector<cv::Point3f>& objectVertices, std::vector<std::vector<int>>& objectFaces, cv::Mat& frame);
int OverlayObjectOnChessboardCenterScaled(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const cv::Vec3d& rvec, cv::Vec3d& tvec, std::vector<cv::Point3f>& objectVertices, const std::vector<std::vector<int>>& objectFaces, cv::Mat& frame, const int numSquaresX, const int numSquaresY, const float squareSize);

#endif //PROJECT4_UTILS_H
