#include "opencv2/opencv.hpp"
#include "aruco.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
using namespace cv;
/*
void createArucoMarkers()
{
    Mat outputMarker;
    aruco::Dictionary markerDictionary = aruco::Dictionary::loadPredefined("ARUCO_MIP_36h12");
    for(int i  = 0; i < 50; ++i )
    {
        aruco::d

    }

}
*/
const float SQUARE_SIZE = 0.02f; // meters 
const Size PATTERN_SIZE = Size(7,9); // number of intersections
const int fps = 20; // fps

void generateObjectPoints(Size boardSize, float squareEdgeLength, std::vector<Point3f>& corners) 
{
    // real positions of the intersections in the 3D world
    for (int i = 0; i < boardSize.height; ++i )
    {
        for(int j = 0; j < boardSize.width; ++j)
        {
            corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f)); // x,y,z coordinates of the intersections in the board coordinate system (z=0 always)
        }
    }
    std::cout << "Object points has been generated!" << std::endl;
}

void getImagePoints(std::vector<Mat>& images, std::vector<std::vector<Point2f>>& allFoundCorners,Size boardSize, bool showResults = false ) 
{
    // detecting intersections from the presaved pictures using the built-in OpenCV algorythm
    int count = 0; // the number of the image being proccesed
    for (auto img : images) // range-based loop iteration
    {
        std::vector<Point2f> pointBuf;
        bool found = findChessboardCorners(img, boardSize, pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE); // the flags basically do grayscale + gammacorrection
        if(found)
        {
            allFoundCorners.push_back(pointBuf);
            ++count;
            std::cout <<"The image points from the "<< count << " frame has been calculated!"<< std::endl;
        }
        if(showResults)
        {
            drawChessboardCorners(img, boardSize, pointBuf, found );
            imshow("IntersectionRecognition", img);
            waitKey();
        }
    }
    
}

void cameraCalibration(std::vector<Mat>& calibrationImages, Mat& cameraMatrix, Mat& distortionCoeff, bool outputExtrinsics = false)
{
    // PATTERN_SIZE and SQUARE_SIZE is global constants
    std::vector<std::vector<Point2f>> imagePoints; // will store the points recognized from the each video frame by getImagePoints function
    getImagePoints(calibrationImages, imagePoints, PATTERN_SIZE, false);
    std::vector<std::vector<Point3f>> objectPoints(1); // vector of vectors, with size of 1 element
    generateObjectPoints(PATTERN_SIZE, SQUARE_SIZE, objectPoints[0]); 
    objectPoints.resize(imagePoints.size(),objectPoints[0]); // makes objectPoints vectorthe same size as imagePoints, and initializes all the elemets with the same value objectPoints[0]
    distortionCoeff = Mat::zeros(8,1, CV_64F); // filling matrix with zeros instead of garbage values (good practice)
    std::vector<Mat> rVecs, tVecs; // extrinsics of the camera: rotation matrix and translation vector
    std::cout << "Please wait for the algorithm to finish calibration..." << std::endl;
    calibrateCamera(objectPoints, imagePoints, PATTERN_SIZE, cameraMatrix, distortionCoeff, rVecs, tVecs);
    std::cout << "Camera has been succecfully calibrated!" << std::endl;
    /*
    if (outputExtrinsics)
    {               

    }
    */
}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distortionCoeff)
{
    remove(name.c_str()); // delete the file if it already exists
    std::ofstream outStream(name);
    auto lambdaLoop = [&outStream](Mat& matrx)
    {
        uint16_t rows = matrx.rows;
        uint16_t cols = matrx.cols;
        for(int r = 0; r < rows; ++r) // writing Intrinsics Camera Matrix
        {
            for(int c = 0; c < cols; ++c)
            {
                double temp = matrx.at<double>(r,c);
                outStream << temp << "\t";

            }   
            outStream << "\n";
        }
    }; // end of lambda function

    if(outStream)
    {
        outStream << "Camera Matrix (Intrinsics: Fx, Fy, Cx, Cy)" << "\n";
        lambdaLoop(cameraMatrix);
        outStream << "Distortion Coefficients (k1 k2 p1 p2 k3)" << "\n";
        lambdaLoop(distortionCoeff);
        outStream.close();
        std::cout << "Calibartion parameters has been saved! Press <Esc> to close Camera window." << std::endl;
        return true; // data has been succesfully written to the file
    }
    return false; // some problems with writing out to a file
}

int main(int argc, char** argv)
{
    std::cout << "Welcome to the Camera Calibration Program!" << std::endl;
    std::cout << "Enter a filename to which will be saved calibration parameters:"<<std::endl;
    std::string filename;
    std::getline(std::cin, filename);
    std::cout << "SPACEBAR - capture frame for calibration" << "\n" << "ENTER - start calibration" << "\n" << "ESC - stop video stream and close the program" << std::endl;
    Mat frame; // current video frame to process
    Mat drawToFrame; // overlays frame if intersection found and OpenCv draws colored lines 
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F); // intrinsics parameter matrix
    Mat distortionCoef; // distorion coefficients to compensate for radial and tangetial distortion
    std::vector<Mat> calibImages; // save good frames for calibration
    std::vector<std::vector<Point2f>> intersections, rejectedCandidates; // recognized intersections
    VideoCapture vid(1);
    if (!vid.isOpened())
    {
        std::cerr << "Error capturing the video" << std::endl;
        getchar();
        return -1;
    }
    namedWindow("Camera", WINDOW_AUTOSIZE); // window where video stream from the camera will be shown
    std::vector<Vec2f> foundPoints; // to store found points
    bool found = false; 
    for(;;)
    {
        if(!vid.read(frame))
            break;
        bool found = findChessboardCorners(frame, PATTERN_SIZE, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE); // the flags basically do grayscale + gammacorrection
        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame, PATTERN_SIZE, foundPoints, found); // draws colored lines into drawToFrame
        if(found)
            imshow("Camera", drawToFrame);
        else
            imshow("Camera", frame);
        char character = waitKey(1000/fps); // waits 20 ms for user to press the specific key, then continue capturing images

        #if defined(WIN32) && !defined(UNIX)
        // platform specific key codes
        switch(character)
        {
            case ' ': 
                //saving image (press spacebar)
                if (found)
                {
                    Mat temp;
                    frame.copyTo(temp);
                    calibImages.push_back(temp);
                    std::cout << "The image has been saved. Total number of images for calibration: "<<calibImages.size()<<std::endl;
                }
                else
                {
                    std::cout << "Algorythm cannot detect the intersection, try to change the position of the pattern" << std::endl;
                }
                break;
            case 13:
                //start calibration (press Enter)
                if (calibImages.size() > 15)
                {
                    cameraCalibration(calibImages, cameraMatrix, distortionCoef, false);
                    saveCameraCalibration(filename + ".txt", cameraMatrix, distortionCoef);
                }
                else
                {
                    std::cout << "Not enough images for calibration, continue capturing the images." << std::endl;
                }
                break;
            case 27:
                //exit at any moment (press Esc)
                vid.release();
                std::cout<<"Video stream has stopped!"<<std::endl;
                return 0;
                break;
                
        }
        #endif
    }
    return 0;
}