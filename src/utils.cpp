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
    bool foundCorners = findChessboardCorners(src, targetSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

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

//     Marker detection variables
     cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
     std::vector<int> markerIds;
     std::vector<std::vector<cv::Point2f>> markerCorners,rejectedCandidates;
     cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();
     cv::aruco::ArucoDetector detector(dictionary,parameters);

     // Detect markers
     detector.detectMarkers(src,markerCorners,markerIds,rejectedCandidates);
    

//    std::vector<int> markerIds;
//    std::vector<std::vector<cv::Point2f>> markerCorners,rejectedCandidates;
//    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//
//    // Detect markers
//    cv::aruco::detectMarkers(src,dictionary, markerCorners,markerIds);

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
                // vertices.push_back(cv::Point3f(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3])));
                vertices.push_back(cv::Point3f(std::stof(tokens[1]),-1 * std::stof(tokens[3]),(std::stof(tokens[2]))));

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

// draw the four outside corners of the chessboard and the 3D axes at the origin of the chessboard on the frame
// cameraMatrix: the camera matrix
// distCoeffs: the distortion coefficients
// rvec: the rotation vector
// tvec: the translation vector
// frame: the frame to draw on
// return: 0 if successful, -1 if error
int overlayCorners(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Vec3d rvec, cv::Vec3d tvec, cv::Mat &frame){
    // Validate the inputs to ensure they contain data
    if (cameraMatrix.empty() || distCoeffs.empty() || frame.empty()) {
        std::cerr << "Error: Input camera matrix, distortion coefficients, or frame is empty." << std::endl;
        return -1;
    }

    // Define the 3D points for the chessboard corners, axes, and masking rectangle
    std::vector<cv::Point3f> objectPoints = {
        // Chessboard corners
        {0, 0, 0}, {8, 0, 0}, {8, -5, 0}, {0, -5, 0},
        // Axes points
        {2, 0, 0}, {0, 2, 0}, {0, 0, 2},
        // Masking rectangle points
        {-1, 1, 0}, {9, 1, 0}, {9, -6, 0}, {-1, -6, 0}
    };

    // Project the 3D points onto the image plane
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    // Draw the masking rectangle over the chessboard
    std::vector<cv::Point> maskingRectangle(imagePoints.begin() + 7, imagePoints.begin() + 11);
    cv::fillPoly(frame, std::vector<std::vector<cv::Point>>{maskingRectangle}, cv::Scalar(255, 255, 255));

    // Illustrate the chessboard corners with circles
    for (int i = 0; i < 4; ++i) {
        cv::circle(frame, imagePoints[i], 6, cv::Scalar(0, 0, 0), cv::FILLED);
    }

    // Depict the 3D axes emanating from the chessboard origin
    cv::line(frame, imagePoints[0], imagePoints[4], cv::Scalar(0, 0, 255), 3); // X-axis in red
    cv::line(frame, imagePoints[0], imagePoints[5], cv::Scalar(0, 255, 0), 3); // Y-axis in green
    cv::line(frame, imagePoints[0], imagePoints[6], cv::Scalar(255, 0, 0), 3); // Z-axis in blue

    return 0;
}

// draw the object on the frame
// cameraMatrix: the camera matrix
// distCoeffs: the distortion coefficients
// rvec: the rotation vector
// tvec: the translation vector
// vertices: the vertices of the object
// faces: the faces of the object
// frame: the frame to draw on
// return: 0 if successful, -1 if error
int overlayObject(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Vec3d& rvec, cv::Vec3d& tvec, std::vector<cv::Point3f>& objectVertices, std::vector<std::vector<int>>& objectFaces, cv::Mat& frame) {
    // Validate input matrices and vectors to ensure they are not empty.
    if (cameraMatrix.empty() || distCoeffs.empty() || objectVertices.empty() || objectFaces.empty() || frame.empty()) {
        std::cerr << "Error: One or more inputs are empty in OverlayObject" << std::endl;
        return -1;
    }
    int numSquaresX = 9.0f;
    int numSquaresY = 6.0f;
    float squareSize = 1.0f;
    float chessboardWidth = (numSquaresX - 1) * squareSize;
    float chessboardHeight = (numSquaresY - 1) * squareSize;
    // Assuming the chessboard lies flat on the XY plane, z is 0.
    // float centerZ = 0.0f;
    // std::cout << "CenterX: " << centerX << " CenterY: " << centerY << std::endl;
    // // Adjust tvec to position the object at the center of the chessboard.
    // tvec[0] -= centerX;
    // tvec[1] -= centerY;
    // // tvec[2] += centerZ;
    // Project object vertices onto the image plane.
    cv::Vec3d chessboardCenter(chessboardWidth / 2.0, chessboardHeight / 2.0, 0);

    // Adjust the translation vector with the chessboard center
    // tvec += chessboardCenter;

    // Verify and correct the orientation of the axes if necessary
    // Make sure that rvec represents the correct rotation for the chessboard orientation
    // Use cv::Rodrigues to convert rvec to a rotation matrix if needed to analyze the rotation
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectVertices, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    // Iterate through each face of the object to draw it on the frame.
    for (const auto& face : objectFaces) {
        // Calculate the average depth (z coordinate) for the face to determine its color.
        float averageDepth = 0.0f;
        for (int vertexIndex : face) {
            averageDepth += objectVertices[vertexIndex].z;
        }
        averageDepth /= face.size();
        int colorIntensity = static_cast<int>((averageDepth / 3.5f) * 155) + 100;
        cv::Scalar faceColor(colorIntensity, colorIntensity, colorIntensity);
        // the four outside corners of the chessboard
        // Draw edges for the current face.
        for (size_t i = 0; i < face.size(); ++i) {
            cv::line(frame, imagePoints[face[i]], imagePoints[face[(i + 1) % face.size()]], faceColor, 2);
            // cv::line(frame,imagePoints[face[i]-1],imagePoints[face[i+1]-1],cv::Scalar(255,255,0), 1);

        }
        
    }

    return 0;
}


int OverlayObjectOnChessboardCenterScaled(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const cv::Vec3d& rvec, cv::Vec3d& tvec, std::vector<cv::Point3f>& objectVertices, const std::vector<std::vector<int>>& objectFaces, cv::Mat& frame, const int numSquaresX, const int numSquaresY, const float squareSize) {
    if (cameraMatrix.empty() || distCoeffs.empty() || objectVertices.empty() || objectFaces.empty() || frame.empty()) {
        std::cerr << "Error: One or more inputs are empty." << std::endl;
        return -1;
    }


    // Find the bounding box of the object
    cv::Point3f minPoint = objectVertices[0];
    cv::Point3f maxPoint = objectVertices[0];
    for (const auto& vertex : objectVertices) {
        minPoint.x = std::min(minPoint.x, vertex.x);
        minPoint.y = std::min(minPoint.y, vertex.y);
        minPoint.z = std::min(minPoint.z, vertex.z);
        
        maxPoint.x = std::max(maxPoint.x, vertex.x);
        maxPoint.y = std::max(maxPoint.y, vertex.y);
        maxPoint.z = std::max(maxPoint.z, vertex.z);
    }

    float objectWidth = maxPoint.x - minPoint.x;
    float objectHeight = maxPoint.y - minPoint.y;
    // Optionally include depth (z-axis) in scaling if desired:
    // float objectDepth = maxPoint.z - minPoint.z;

    float chessboardWidth = (numSquaresX - 1) * squareSize;
    float chessboardHeight = (numSquaresY - 1) * squareSize;

    // Calculate scale factors
    float scaleX = chessboardWidth / objectWidth;
    float scaleY = chessboardHeight / objectHeight;
    // For uniform scaling in all dimensions, find the minimum scale factor
    float scale = std::min(scaleX, scaleY);
    // float scale = 1.2;
    std::cout << "Scale: " << scale << std::endl;

    // Now, apply this scale to all vertices
    for (auto& vertex : objectVertices) {
        vertex.x *= scale;
        vertex.y *= scale;
        vertex.z *= scale; // Scale z if you want to maintain the object's 3D aspect ratio
    }
    // // Calculate the chessboard center in world coordinates.
    // float chessboardWidth = (numSquaresX - 1) * squareSize;
    // float chessboardHeight = (numSquaresY - 1) * squareSize;

    // // Calculate object's bounding box to determine its current size.
    // auto [minVertex, maxVertex] = std::minmax_element(objectVertices.begin(), objectVertices.end(), [](const cv::Point3f& a, const cv::Point3f& b) {
    //     return a.x < b.x; // Compare based on the x-coordinate.
    // });
    // float objectWidth = maxVertex->x - minVertex->x;
    // float objectHeight = std::max_element(objectVertices.begin(), objectVertices.end(), [](const cv::Point3f& a, const cv::Point3f& b) {
    //     return a.y < b.y; // Compare based on the y-coordinate.
    // })->y - std::min_element(objectVertices.begin(), objectVertices.end(), [](const cv::Point3f& a, const cv::Point3f& b) {
    //     return a.y < b.y;
    // })->y;

    // // Calculate the scale factor to fit the object within the chessboard dimensions.
    // float scaleX = chessboardWidth / objectWidth;
    // float scaleY = chessboardHeight / objectHeight;
    // float scale = std::min(scaleX, scaleY); // Use the smaller scale to ensure the object fits within the chessboard.
    // std::cout << "Scale: " << scale << std::endl;
    // // Scale the object vertices.
    // for (auto& vertex : objectVertices) {
    //     vertex.x *= scale;
    //     vertex.y *= scale;
    //     // Optionally scale z if you want to maintain the aspect ratio, or leave it to adjust depth independently.
    // }

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectVertices, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    for (const auto& face : objectFaces) {
        float averageDepth = 0.0f;
        for (int vertexIndex : face) {
            averageDepth += objectVertices[vertexIndex].z;
        }
        averageDepth /= face.size();
        int colorIntensity = static_cast<int>((averageDepth / 3.5f) * 155) + 100;
        cv::Scalar faceColor(colorIntensity, colorIntensity, colorIntensity);

        for (size_t i = 0; i < face.size(); ++i) {
            cv::line(frame, imagePoints[face[i]], imagePoints[face[(i + 1) % face.size()]], faceColor, 3);
        }
    }

    return 0;
}
void initCalibration(Mat &Cam, Mat &DistCoeffs, Mat &src){
    Cam = Mat::eye(3, 3, CV_64F);
    Cam.at<double>(0, 2) = src.cols/2;
    Cam.at<double>(1, 2) = src.rows/2;
    DistCoeffs = Mat::zeros(5, 1, CV_64F);
    cout << "Camera Matrix Before Calibration" << endl;
    mprint(Cam);
    cout << "Distortion Matrix Before Calibration" << endl;
    mprint(DistCoeffs);
}

int save2xml(Mat &Cam, cv::Mat &DistCoeffs, string path) {
    cv::FileStorage fs
            (path,
             cv::FileStorage::WRITE);

    // Write the matrices to the file
    fs << "Camera_Matrix" << Cam;
    fs << "Distortion_Coefficients" << DistCoeffs;

    // Release the file storage
    fs.release();

    return 0;
}