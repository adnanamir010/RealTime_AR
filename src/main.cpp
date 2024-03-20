/*
 * Adnan Amir and Srijan Dokania
 * CS 5330 PRCV
 * Spring 2024
 * Purpose of file: Source file for entire AR system
 */

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "opencv2/aruco.hpp"
#include "../include/utils.h"
#include <stdio.h>

using namespace std;
using namespace cv;

enum mode {
    SHOW_CORNERS = 1,
    CALLIBRATION = 2,
    CAMERA_POSE = 3,
} mode;


const int menu_height = 100;
const int menu_width = 300;

const int button_height = 20;
const int button_width = 300;
char targetPath[256];
const string menu_options[] = {
	"Show Corners",
	"Calibrate Camera",
	"Calculate Outside Corners and Show 3D Axis",
	"Show Virtual Object",
	"Exit"
};
void showCorners() {
    // Code to show corners
    cout << "Showing corners..." << endl;
}

void calibrateCam() {
    // Code to calibrate camera
    cout << "Calibrating camera..." << endl;
}

void findCameraPose() {
    // Code to find camera pose
    cout << "Finding camera pose..." << endl;
}
void showVirtualObject() {
    // Code to show virtual object
    cout << "Showing virtual object..." << endl;
}
// Function to draw styled buttons for the interactive menu
void drawStyledMenu(Mat& menuImage) {
    menuImage.setTo(Scalar(100, 100, 100)); // Set a darker grey background

    int buttonSpacing = 15; // Space between buttons
    int buttonHeight = (menuImage.rows - (buttonSpacing * (sizeof(menu_options) / sizeof(string) + 1))) / (sizeof(menu_options) / sizeof(string));
    int buttonWidth = menuImage.cols - (2 * buttonSpacing);
    int yPos = buttonSpacing;

    for (const auto& option : menu_options) {
        // Draw the button
        Point topLeft(buttonSpacing, yPos);
        Point bottomRight(buttonSpacing + buttonWidth, yPos + buttonHeight);
        rectangle(menuImage, topLeft, bottomRight, Scalar(200, 200, 100), FILLED);
        
        // Draw the button text
        int baseLine;
        Size textSize = getTextSize(option, FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseLine);
        Point textOrg((menuImage.cols - textSize.width) / 2, yPos + (buttonHeight + textSize.height) / 2);
        putText(menuImage, option, textOrg, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1.5);

        yPos += buttonHeight + buttonSpacing;
    }
}
// Callback function for mouse events
void onMouse(int event, int x, int y, int, void*) {
    if (event != EVENT_LBUTTONDOWN) return; // Only interested in left button clicks

    int buttonSpacing = 10;
    int totalOptions = sizeof(menu_options) / sizeof(string);
    int menuHeight = 400; // Assuming a menu height, adjust based on your initialization
    int buttonHeight = (menuHeight - (buttonSpacing * (totalOptions + 1))) / totalOptions;

    int clickedOption = -1;
    for (int i = 0; i < totalOptions; ++i) {
        int yPos = buttonSpacing + i * (buttonHeight + buttonSpacing);
        if (y > yPos && y < yPos + buttonHeight) {
            clickedOption = i;
            break;
        }
    }

    if (clickedOption != -1) {
        cout << "Selected: " << menu_options[clickedOption] << endl;
        // Handle menu option selection here
    
			switch (clickedOption) {
			case 0: {
                // saveFeatures("olympus", "baseline");
				showCorners();
				break;
			}
			case 1: {
                // saveFeatures("olympus", "histogram_matching");
				calibrateCam();
				break;
			}
			case 2: {
                // saveFeatures("olympus", "multi-histogram_matching");
				findCameraPose();
				break;
			}
			case 3: {
                // saveFeatures("olympus", "texture-color_matching");
				showVirtualObject();
				break;
			}
			case 4: {
				exit(0);
				break;
			}
			}
		}

}


int main(int argc, char *argv[]) {

//    if (argc < 2) {
//        printf("usage %s <image filename> \n", argv[0]);
//        exit(-1);
//    }
    // string path = "E:\\backup\\Desktop\\College\\NEU\\sem 2\\cs 5330 pattern recognition and computer vision\\Project4\\assets\\Sentinels_Esports_Logo.png";
    string path = "assets/Sentinels_Esports_Logo.png";
    cv::VideoCapture *capdev;
    string path2 = "assets/cow-nonormals.obj";
    // open the video device
    capdev = new cv::VideoCapture(2);
    capdev->set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capdev->set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    if( !capdev->isOpened() ) {
        printf("Unable to open video device\n");
        return(-1);
    }

    // get some properties of the image
    cv::Size refS( (int) capdev->get(cv::CAP_PROP_FRAME_WIDTH ),
                   (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    cv::Mat src,dst,cbDistCoeffs, arucoDistCoeffs,cbCam,arucoCam;
    Size chessboardSize(9, 6); //Size of the Chessboard target
    Size arucoSize(5, 7); //Size of the Arucoboard target
    vector<vector<Point2f>> cbCornerList;
    vector<vector<Vec3f>> cbPointList;
    vector<vector<Point2f>> arucoCornerList;
    vector<vector<Vec3f>> arucoPointList;
    vector<Mat> cbR, cbT, arucoR, arucoT;
    int frameCount=0;

    Mat overlay = imread(path,IMREAD_COLOR);
    std::vector<cv::Point3f> vertices;
    std::vector<cv::Point3f> normals;
    std::vector<std::vector<int>> facevertices;
    parseOBJFile(path2, vertices, normals, facevertices);


    cv::Point3f minPoint = vertices[0];
    cv::Point3f maxPoint = vertices[0];
    for (const auto& vertex : vertices) {
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
    int numSquaresX = 9.0;
    int numSquaresY = 6.0;
    float squareSize = 1.0;
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
    for (auto& vertex : vertices) {
        vertex.x *= scale;
        vertex.y *= scale;
        vertex.z *= scale; // Scale z if you want to maintain the object's 3D aspect ratio
    }



    bool overlayFlag = false;
    bool cbCalibrationFlag = false;
    bool arcuroCalibrationFlag = false;
    bool objectFlag = false;

    mode = SHOW_CORNERS;

    //initializing 3D world coordinate vectors for targets
    vector<Vec3f> cbPoints = initWorldCoords(chessboardSize);
    vector<Vec3f> arucoPoints = initWorldCoords(arucoSize);



    for(;;) {
        *capdev >> src; // get a new frame from the camera, treat as a stream
        if( src.empty() ) {
            printf("frame is empty\n");
            break;
        }
        // resize the frame to 1/2 of the original size
        // resize(src, src, Size(), 0.5, 0.5);
        dst = src.clone();

        // see if there is a waiting keystroke
        char key = cv::waitKey(10);

        //get the chessboard and aruco corners
        vector<Point2f> cbCorners,arucoCorners,outer4;
        bool foundcb = getChessboardCorners(src,cbCorners,chessboardSize);
        bool foundAruco = getArucoCorners(src,arucoCorners,outer4);



        if( key == 'q') {
            capdev->release();
            break;
        }
        if (key == 'o') {
            overlayFlag = !overlayFlag;
        }
        if (key == 'r'){
            objectFlag = !objectFlag;
        }

        switch (key) {
            case ' ':mode = SHOW_CORNERS;
                break;
            case 'c':{
                mode = CALLIBRATION;
                frameCount = 0;
                break;
            }
            case 'p':mode = CAMERA_POSE;
                break;
        };

        switch (mode) {
            case SHOW_CORNERS:{
                if (foundcb){
                    vector<Point2f> outside;
                    int idx[] = {0,8,45,53};
                    for (int i: idx){
                        outside.push_back(cbCorners[i]);
                    }
                    drawChessboardCorners(dst,chessboardSize,cbCorners,foundcb);
                    if (overlayFlag){
                        overlayImage(overlay, dst, outside);
                    }
                    if (objectFlag){
                        // calculate the pose of the chessboard
                        cv::Vec3d rvec, tvec;
                        cv::solvePnP(cbPoints, cbCorners, cbCam, cbDistCoeffs, rvec, tvec);

                        // print the pose of the chessboard
                        std::cout << "rvec: " << rvec << std::endl;
                        std::cout << "tvec: " << tvec << std::endl;
                        cv::Mat rotationMatrix;
                        cv::Rodrigues(rvec, rotationMatrix);

                        // Calculate the chessboard's center in its own reference frame
                        cv::Point3f chessboardCenter(4.0, -2.5, 0.0);

                        // Transform the chessboard center to the camera's reference frame
                        cv::Mat chessboardCenterMat = (cv::Mat_<double>(3,1) << chessboardCenter.x, chessboardCenter.y, chessboardCenter.z);
                        chessboardCenterMat = rotationMatrix * chessboardCenterMat + cv::Mat(tvec);

                        // Use the transformed chessboard center for the object's translation vector
                        cv::Vec3d tvec_centered = cv::Vec3d(chessboardCenterMat);

                        // draw the four outside corners of the chessboard as circles
                        // and the 3D axes at the origin of the chessboard
                        overlayCorners(cbCam, cbDistCoeffs, rvec, tvec, dst);
                        overlayObject(cbCam, cbDistCoeffs, rvec, tvec_centered, vertices, facevertices, dst);
                        // OverlayObjectOnChessboardCenterScaled(cbCam, cbDistCoeffs, rvec, tvec, vertices, facevertices, dst, chessboardSize.width, chessboardSize.height, 0.5);
                    }

                }
                if (foundAruco){
                    for (int i=0; i<arucoCorners.size();i++){
                        circle(dst,arucoCorners[i],1,Scalar(0,0,255),5);
                    }
                    if (overlayFlag){
                        overlayImage(overlay, dst, outer4);
                    }
                }
                break;
            }
            case CALLIBRATION:{
                if (frameCount <= 6){

                    if (foundcb){
                        drawChessboardCorners(dst,chessboardSize,cbCorners,foundcb);
                        cbCornerList.push_back(cbCorners);
                        cbPointList.push_back(cbPoints);
                    }
                    if (foundAruco){
                        for (int i=0; i<arucoCorners.size();i++){
                            circle(dst,arucoCorners[i],2,Scalar(0,0,255));
                        }
                        arucoCornerList.push_back(arucoCorners);
                        arucoPointList.push_back(arucoPoints);
                    }
                }
                else{
                    mode = SHOW_CORNERS;
                    if (foundcb) {
                        double cbRepError = calibrateCamera(cbPointList, cbCornerList, Size(src.rows, src.cols),
                                                            cbCam,cbDistCoeffs, cbR, cbT);
                        cout << "Chessboard Camera Matrix" << endl;
                        mprint(cbCam);
                        cout << "Chessboard Distortion Matrix" << endl;
                        mprint(cbDistCoeffs);
                        cout << "Chessboard Reprojection Error: " << cbRepError << endl;
                        cbCalibrationFlag = true;
                        cout << "Chessboard Callibration Complete" << endl;

                    }
                    if (foundAruco){
                        double arucoRepError = calibrateCamera(arucoPointList,arucoCornerList,Size(src.rows,src.cols),
                                                               arucoCam,arucoDistCoeffs,arucoR,arucoT);
                        cout << "Arucoboard Camera Matrix" << endl;
                        mprint(arucoCam);
                        cout << "Arucoboard Distortion Matrix" << endl;
                        mprint(arucoDistCoeffs);
                        cout << "Arucoboard Reprojection Error: " << arucoRepError << endl;
                        arcuroCalibrationFlag = true;
                        cout << "Arucoboard Callibration Complete" << endl;

                    }


                    cout << "Callibration Complete. Press c to recallibrate" << endl;
                }
                frameCount++;
                break;
            }
            case CAMERA_POSE:{
                if (cbCalibrationFlag || arcuroCalibrationFlag) {
                    if (foundcb && cbCalibrationFlag) {
                        vector<Point2f> outside;
                        int idx[] = {0, 8, 45, 53};
                        for (int i: idx) {
                            outside.push_back(cbCorners[i]);
                        }
                        drawChessboardCorners(dst, chessboardSize, cbCorners, foundcb);
                        if (overlayFlag) {
                            overlayImage(overlay, dst, outside);
                        }
                        Mat R,T;
                        bool status = solvePnPRansac(cbPoints,cbCorners,cbCam,cbDistCoeffs,R,T);
                        if (status){
                            rad2deg(R);
                            cout << "Chessboard Rotation Matrix:" << endl;
                            mprint(R);
                            cout << "Chessboard Translation Matrix:" << endl;
                            mprint(T);
                        }
                    }
                    if (foundAruco && arcuroCalibrationFlag) {
                        for (int i = 0; i < arucoCorners.size(); i++) {
                            circle(dst, arucoCorners[i], 1, Scalar(0, 0, 255), 5);
                        }
                        if (overlayFlag) {
                            overlayImage(overlay, dst, outer4);
                        }
                        Mat R,T;
                        bool status = solvePnPRansac(arucoPoints,arucoCorners,arucoCam,arucoDistCoeffs,R,T);
                        if (status){
                            rad2deg(R);
                            cout << "Aruco Rotation Matrix:" << endl;
                            mprint(R);
                            cout << "Aruco Translation Matrix:" << endl;
                            mprint(T);
                        }


                    }
                }
                else{
                    mode = SHOW_CORNERS;
                    if (!cbCalibrationFlag){
                        cout << "Please Calibrate on Chessboard First. Press c to calibrate" << endl;
                    }
                    if (!arcuroCalibrationFlag){
                        cout << "Please Calibrate on Arucoboard First. Press c to calibrate" << endl;
                    }
                }
                break;
            }
        };
        cv::namedWindow("Video", 1); // identifies a window
        putText(dst, //target image
                to_string(cbCornerList.size()), //text
                cv::Point(10, dst.rows / 10), //top-left position
                cv::FONT_HERSHEY_TRIPLEX,
                1.0,
                CV_RGB(118, 185, 0), //font color
                2);
        cv::imshow("Video", dst);

    }
    delete capdev;
    Mat menuImage = Mat::zeros(400, 240, CV_8UC3); // Adjust size as needed

    drawStyledMenu(menuImage);

    namedWindow("RealTime Augmented Reality", WINDOW_AUTOSIZE);
    setMouseCallback("RealTime Augmented Reality", onMouse);

    imshow("RealTime Augmented Reality", menuImage);
    waitKey(0); // Wait indefinitely until a user interacts



    return(0);
}