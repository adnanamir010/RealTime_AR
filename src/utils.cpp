/*
 * Adnan Amir and Srijan Dokania
 * CS 5330 PRCV
 * Spring 2024
 * Purpose of file: Source file for various utility functions
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include "../include/utils.h"
#include "opencv2/aruco.hpp"
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using namespace cv;

vector<Vec3f> initWorldCoords(Size targetSize) {
    vector<Vec3f> points;
    for (int i = 0; i < targetSize.height; i++) {
        for (int j = 0; j < targetSize.width; j++) {
            // Create a coordinate with x and y positions matching the loop indices
            // Z-coordinate is set to 0 to create a 2D plane in 3D space
            Vec3f coordinates = Vec3f(j, -i, 0);
            points.push_back(coordinates); // Add the coordinate to the points vector
        }
    }
    return points; // Return the populated vector of points
}


bool getChessboardCorners(Mat &src, vector<Point2f> &corners, Size targetSize) {
    // Attempt to find the corners of a chessboard pattern in the source image
    bool foundCorners = findChessboardCorners(src, targetSize, corners);

    if (foundCorners) { // If corners are found
        Mat gray;
        cvtColor(src, gray, COLOR_BGR2GRAY); // Convert the source image to grayscale for cornerSubPix function
        Size subPixWinSize(10, 10); // Define the window size for corner refinement
        TermCriteria termCrit(TermCriteria::COUNT|TermCriteria::EPS, 30, 0.01); // Criteria for cornerSubPix refinement
        cornerSubPix(gray, corners, subPixWinSize, Size(-1, -1), termCrit); // Refine the located corners
    }
    return foundCorners; // Return whether corners were found
}

bool getArucoCorners(cv::Mat& src, std::vector<cv::Point2f>& corners,vector<Point2f> &outer4) {
    // Pre-size the corners and outer4 vectors to expected sizes
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

    // Assign the top left corner of detected markers to the corners vector based on marker ID
    for (size_t i = 0; i < markerIds.size(); i++) {
        int idx = markerIds[i];
        if (idx < corners.size()) {
            corners[idx] = markerCorners[i][0];
        }
    }

    // Identify specific markers to use as outer corners of the Aruco board
    for (size_t i = 0; i < markerIds.size(); i++) {
        int idx = markerIds[i];
        if (idx == 0) outer4[0] = markerCorners[i][0];
        else if (idx == 4) outer4[1] = markerCorners[i][1];
        else if (idx == 30) outer4[2] = markerCorners[i][3];
        else if (idx == 34) outer4[3] = markerCorners[i][2];
    }


    return markerCorners.size() == 35; // successfully extract Aruco corners
}

void overlayImage(Mat &src, Mat &dst, vector<Point2f> &corners) {
    // Define source image corners for homography calculation
    vector<Point2f> srcCorners = { Point2f(0, 0), Point2f(src.size().width, 0),
                                   Point2f(0, src.size().height),Point2f(src.size().width, src.size().height)};


    // Calculate the homography matrix to map srcCorners to corners
    Mat homography = findHomography(srcCorners, corners);

    // Warp the source image to the destination perspective
    Mat warpedSrc;
    warpPerspective(src, warpedSrc, homography, dst.size());

    // Iterate through the warped source image to overlay non-black pixels
    for (int i = 0; i < warpedSrc.rows; i++) {
        for (int j = 0; j < warpedSrc.cols; j++) {
            // Only overlay non-black pixels
            if (warpedSrc.at<Vec3b>(i, j) != Vec3b(0, 0, 0)) {
                dst.at<Vec3b>(i, j) = warpedSrc.at<Vec3b>(i, j);
            }
        }
    }
}

void mprint(cv::Mat x) {
    // Iterate through each element in the matrix and print it
    for (int i = 0; i < x.rows; i++) {
        for (int j = 0; j < x.cols; j++) {
            cout << x.at<double>(i, j) << ", ";
        }
        cout << "\n";
    }
}


void rad2deg(cv::Mat x) {
    // Iterate through each element in the matrix and convert it from radians to degrees
    for (int i = 0; i < x.rows; i++) {
        for (int j = 0; j < x.cols; j++) {
            x.at<double>(i, j) *= 180 / M_PI;
        }
    }
}


std::vector<std::string> split(const std::string &str, char delim) {
    // Use stringstream to tokenize the string based on the delimiter
    std::vector<std::string> tokens;
    std::stringstream str_stream(str);
    std::string part;

    // Extract each token and add it to the tokens vector
    while (std::getline(str_stream, part, delim)) {
        tokens.push_back(part);
    }

    return tokens; // Return the tokenized parts
}

void parseOBJFile(const std::string &fName, std::vector<cv::Point3f> &vertices, std::vector<cv::Point3f> &normals, std::vector<std::vector<int>> &faceVertices) {
    std::string currentLine;
    std::ifstream inFile(fName);

    if (!inFile.is_open()) { // Check if the file could be opened
        std::cout << "Fail to open: " << fName << std::endl;
        return;
    }

    // Read each line from the file
    while (std::getline(inFile, currentLine)) {
        // Split the current line by spaces into tokens
        auto tokens = split(currentLine, ' ');

        // Ensure there's at least one token to process
        if (!tokens.empty()) {
            // Handle vertices (v)
            if (tokens[0] == "v" && tokens.size() >= 4) {
                // Convert the tokens to floats and add them as a Point3f to the vertices vector
                vertices.push_back(cv::Point3f(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3])));
            }
                // Handle normals (vn)
            else if (tokens[0] == "vn" && tokens.size() >= 4) {
                // Convert the tokens to floats and add them as a Point3f to the normals vector
                normals.push_back(cv::Point3f(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3])));
            }
                // Handle face elements (f)
            else if (tokens[0] == "f" && tokens.size() >= 4) {
                std::vector<int> faceIndices;
                // Iterate through each face token (excluding the first token which is "f")
                for (size_t i = 1; i < tokens.size(); ++i) {
                    // Split the face token by '/' to separate vertex/texture/normal indices
                    auto indices = split(tokens[i], '/');
                    // The first element is the vertex index (note: OBJ indices are 1-based)
                    if (!indices.empty()) {
                        int vertexIndex = std::stoi(indices[0]);
                        faceIndices.push_back(vertexIndex - 1); // Convert to 0-based index
                    }
                }
                // Add the list of vertex indices for this face to the faceVertices vector
                faceVertices.push_back(faceIndices);
            }
        }
    }
    // Close the file after parsing is complete
    inFile.close();
}
