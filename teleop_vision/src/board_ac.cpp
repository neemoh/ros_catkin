/**
 * This tutorial demonstrates simple receipt of messages over the ROS system, using
 * a threaded Spinner object to receive callbacks in multiple threads at the same time.
 */
//============================================================================
// Name        : arucoDetectBoard.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================



#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread.hpp>
using namespace std;
using namespace cv;

namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{l        |       | Marker side lenght (in pixels) }"
        "{s        |       | Separation between two consecutive markers in the grid (in pixels)}"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        ;
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


void drawAC(InputOutputArray _image, InputArray _cameraMatrix, InputArray _distCoeffs,
              InputArray _rvec, InputArray _tvec) {

    CV_Assert(_image.getMat().total() != 0 &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));

    // project axis points
    vector< Point3f > axisPoints;
    axisPoints.push_back(Point3f(400, 450, 0));
    axisPoints.push_back(Point3f(700, 450, 0));
    axisPoints.push_back(Point3f(700, 250, 0));
    axisPoints.push_back(Point3f(400, 250, 0));

    axisPoints.push_back(Point3f(400, 450, 50));
    axisPoints.push_back(Point3f(700, 450, 50));
    axisPoints.push_back(Point3f(700, 250, 50));
    axisPoints.push_back(Point3f(400, 250, 50));

    vector< Point2f > imagePoints;
    projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

    // draw axis lines
    int points[7] = {0,1,2,4,5,6};
    for(int i= 0; i<6; i++){
        line(_image, imagePoints[points[i]], imagePoints[points[i]+1], Scalar(200, 100, 10), 2);
    }
    for(int i= 0; i<4; i++){
        line(_image, imagePoints[i], imagePoints[i+4], Scalar(200, 100, 10), 2);
    }
    line(_image, imagePoints[3], imagePoints[0], Scalar(200, 100, 10), 2);
    line(_image, imagePoints[7], imagePoints[4], Scalar(200, 100, 10), 2);
}


ros::Duration d(0.01);

class Listener
{
public:
  void chatter1(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO_STREAM("chatter1: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
    d.sleep();
  }
  void chatter2(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO_STREAM("chatter2: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
    d.sleep();
  }
  void chatter3(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO_STREAM("chatter3: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
    d.sleep();
  }
};

void chatter4(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("chatter4: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
  d.sleep();
}


/**
 */
int main(int argc, char *argv[]) {
//    CommandLineParser parser(argc, argv, keys);
//    parser.about(about);
//
//    cout << "Hi" << endl;
//    if(argc < 7) {
//        parser.printMessage();
//        return 0;
//    }
	string c = "/home/nearlab/workspace/cameracalib/Debug/out_camera_data.xml";
    int markersX = 9;
    int markersY = 6;
    float markerLength = 100;
    float markerSeparation = 20;
    int dictionaryId = 9;
    int camId = 0;
    bool refindStrategy = false;

    Mat camMatrix, distCoeffs;

        bool readOk = readCameraParameters(c, camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }


    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    VideoCapture inputVideo;
    int waitTime;

        inputVideo.open(camId);
        waitTime = 40;


    float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                               markerSeparation);

    // create board object
    Ptr<aruco::GridBoard> gridboard =
        aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();

    double totalTime = 0;
    int totalIterations = 0;

    while(inputVideo.isOpened()) {
        double tick = (double)getTickCount();

        Mat image, imageCopy;
        inputVideo >> image;

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        Vec3d rvec, tvec;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy)
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix,
                                         distCoeffs);

        // estimate board pose
        int markersOfBoardDetected = 0;
        if(ids.size() > 0)
            markersOfBoardDetected =
                aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);

        // draw results
        image.copyTo(imageCopy);
//        if(ids.size() > 0) {
//            aruco::drawDetectedMarkers(imageCopy, corners, ids);
//        }

//        if(showRejected && rejected.size() > 0)
//            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        if(markersOfBoardDetected > 0){
        	aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

        	drawAC(imageCopy, camMatrix, distCoeffs, rvec, tvec);
        }
        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

    }



//  ros::init(argc, argv, "listener");
//  ros::NodeHandle n;
//
//  Listener l;
//  ros::Subscriber sub1 = n.subscribe("chatter", 10, &Listener::chatter1, &l);
//  ros::Subscriber sub2 = n.subscribe("chatter", 10, &Listener::chatter2, &l);
//  ros::Subscriber sub3 = n.subscribe("chatter", 10, &Listener::chatter3, &l);
//  ros::Subscriber sub4 = n.subscribe("chatter", 10, chatter4);
//
//  /**
//   * The MultiThreadedSpinner object allows you to specify a number of threads to use
//   * to call callbacks.  If no explicit # is specified, it will use the # of hardware
//   * threads available on your system.  Here we explicitly specify 4 threads.
//   */
//  ros::MultiThreadedSpinner s(4);
//  ros::spin(s);

    return 0;
}


