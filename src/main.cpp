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
} mode;


int main(int argc, char *argv[]) {

//    if (argc < 2) {
//        printf("usage %s <image filename> \n", argv[0]);
//        exit(-1);
//    }

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
        vector<Point2f> cbCorners,arucoCorners;
        bool foundcb = getChessboardCorners(src,cbCorners,chessboardSize);
        bool foundAruco = getArucoCorners(src,arucoCorners);



        if( key == 'q') {
            capdev->release();
            break;
        }
        if (key == 's') {
            cv::imwrite("../img/screenshot.png", dst);
        }

        switch (key) {
            case ' ':mode = SHOW_CORNERS;
                break;
            case 'c':{
                mode = CALLIBRATION;
                frameCount = 0;
                cbCorners.clear();
                arucoCorners.clear();
                break;
            }
        };

        switch (mode) {
            case SHOW_CORNERS:{
                if (foundcb){
                    drawChessboardCorners(dst,chessboardSize,cbCorners,foundcb);
                }
                if (foundAruco){
                    for (int i=0; i<arucoCorners.size();i++){
                        circle(dst,arucoCorners[i],2,Scalar(0,0,255));
                    }
                }
                break;
            }
            case CALLIBRATION:{
                if (frameCount <= 50){

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
                        for (int i = 0; i < cbCam.rows; i++) {
                            for (int j = 0; j < cbCam.cols; j++) {
                                cout << cbCam.at<double>(i, j) << ", ";
                            }
                            cout << "\n";
                        }
                        cout << "Chessboard Distortion Matrix" << endl;
                        for (int i = 0; i < cbDistCoeffs.rows; i++) {
                            for (int j = 0; j < cbDistCoeffs.cols; j++) {
                                cout << cbDistCoeffs.at<double>(i, j) << ", ";
                            }
                            cout << "\n";
                        }
                        cout << "Chessboard Reprojection Error: " << cbRepError << endl;

                    }
                    if (foundAruco){
                        double arucoRepError = calibrateCamera(arucoPointList,arucoCornerList,Size(src.rows,src.cols),
                                                               arucoCam,arucoDistCoeffs,arucoR,arucoT);
                        cout << "Arucoboard Camera Matrix" << endl;
                        for (int i = 0; i < arucoCam.rows; i++) {
                            for (int j = 0; j < arucoCam.cols; j++) {
                                cout << arucoCam.at<double>(i, j) << ", ";
                            }
                            cout << "\n";
                        }
                        cout << "Arucoboard Distortion Matrix" << endl;
                        for (int i = 0; i < arucoDistCoeffs.rows; i++) {
                            for (int j = 0; j < arucoDistCoeffs.cols; j++) {
                                cout << arucoDistCoeffs.at<double>(i, j) << ", ";
                            }
                            cout << "\n";
                        }
                        cout << "Arucoboard Reprojection Error: " << arucoRepError << endl;
                    }


                    cout << "Callibration Complete. Press c to recallibrate" << endl;
                }
                frameCount++;
                break;
            }
        };
        cv::namedWindow("Video", 1); // identifies a window
//        putText(dst, //target image
//                name, //text
//                cv::Point(10, dst.rows / 10), //top-left position
//                cv::FONT_HERSHEY_TRIPLEX,
//                1.0,
//                CV_RGB(118, 185, 0), //font color
//                2);
        cv::imshow("Video", dst);

    }

    delete capdev;
    return(0);
}
