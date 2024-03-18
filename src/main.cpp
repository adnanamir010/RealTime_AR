#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "opencv2/aruco.hpp"
#include "../include/utils.h"
#include "../include/obj_utils.h"
#include <stdio.h>

using namespace std;
using namespace cv;

enum mode {
    SHOW_CORNERS = 1,
    CALLIBRATION = 2,
    CAMERA_POSE = 3,
} mode;


int main(int argc, char *argv[]) {

//    if (argc < 2) {
//        printf("usage %s <image filename> \n", argv[0]);
//        exit(-1);
//    }
    string path = "E:\\backup\\Desktop\\College\\NEU\\sem 2\\cs 5330 pattern recognition and computer vision\\Project4\\assets\\Sentinels_Esports_Logo.png";

    cv::VideoCapture *capdev;

    // open the video device
    capdev = new cv::VideoCapture(0);
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
    bool overlayFlag = false;
    bool cbCalibrationFlag = false;
    bool arcuroCalibrationFlag = false;


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
        resize(src, src, Size(), 0.5, 0.5);
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
    return(0);
}