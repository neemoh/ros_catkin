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
#include "geometry_msgs/Pose.h"
#include <boost/thread.hpp>
using namespace std;
using namespace cv;

/**
 */

class boardDetector{
public:
	boardDetector(Mat& _camMatrix, Mat& _distCoeffs){
		markersX = 9;
		markersY = 6;
		markerLength = 100;
		markerSeparation = 20;
		dictionaryId = 9;
        axisLength = 200;
		refindStrategy = false;

		camMatrix = _camMatrix;
		distCoeffs = _distCoeffs;
		markersOfBoardDetected = 0;
		detectorParams = aruco::DetectorParameters::create();
		detectorParams->doCornerRefinement = true; // do corner refinement in markers
		dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
		gridboard =	aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
		board = gridboard.staticCast<aruco::Board>();
	}
	void detect(Vec3d& _rvec, Vec3d& _tvec){
		// detect markers
		aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

		// refind strategy to detect more markers
		if(refindStrategy)
			aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix,distCoeffs);

		// estimate board pose
		if(ids.size() > 0){
			markersOfBoardDetected =
					aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);
			_rvec = rvec;
			_tvec = tvec;
		}
	}
	void drawAxis(){
		if(markersOfBoardDetected > 0)
			aruco::drawAxis(image, camMatrix, distCoeffs, rvec, tvec, axisLength);
	}
	void drawDetectedMarkers(){
		if(ids.size() > 0)
			aruco::drawDetectedMarkers(image, corners, ids);
	}
public:
	int markersX;
	int markersY;
	float markerLength;
	float markerSeparation;
	int dictionaryId;
	float axisLength;
	bool refindStrategy;
	int markersOfBoardDetected;
	Mat image;
	Mat camMatrix, distCoeffs;
    vector<int> ids;
    vector<vector< Point2f > > corners, rejected;
    Vec3d rvec, tvec;
	Ptr<aruco::DetectorParameters> detectorParams;
	Ptr<aruco::Dictionary> dictionary;
	Ptr<aruco::GridBoard> gridboard;
	Ptr<aruco::Board> board;
};
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
	void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
	{
		//    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
		robotPose.position = msg->position;
		robotPose.orientation = msg->orientation;
		d.sleep();
	}
	//  void chatter2(const std_msgs::String::ConstPtr& msg)
	//  {
	//    ROS_INFO_STREAM("chatter2: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
	//    d.sleep();
	//  }
public:
	geometry_msgs::Pose robotPose;
};



/**
 */
int main(int argc, char *argv[]) {

	//-------------------------------    Get image from camera -----------------------------
    int camId = 0;
    Mat camMatrix, distCoeffs;
	string c = "/home/nearlab/workspace/cameracalib/Debug/out_camera_data.xml";
    int waitTime= 40;

    bool readOk = readCameraParameters(c, camMatrix, distCoeffs);
    if(!readOk) {
    	cerr << "Invalid camera file" << endl;
    	return 0;
    }
    VideoCapture inputVideo;
    inputVideo.open(camId);

    double totalTime = 0;
    int totalIterations = 0;

    boardDetector b(camMatrix, distCoeffs);

    //-----------------------------------------------------  ros initialization
	double freq_ros;
    std::string nodename("ACboard");
    ros::init(argc, argv, nodename);
	ros::NodeHandle n(nodename);

	n.param<double>("/vrep_haptics_frequency", freq_ros, 1000);
    ros::Rate loop_rate(freq_ros);

    Listener l;
    ros::Subscriber sub1 = n.subscribe("chatter", 10, &Listener::robotPoseCallback, &l);
	ROS_INFO("Initialization done.");


	while(ros::ok() && inputVideo.isOpened()) {
		double tick = (double)getTickCount();

		Mat imageCopy;
		Vec3d rvec, tvec;

		inputVideo >> b.image;
		b.detect(rvec, tvec);
		b.drawAxis();
		b.drawDetectedMarkers();
		// draw results
		b.image.copyTo(imageCopy);

		if(b.markersOfBoardDetected > 0){
			drawAC(imageCopy, camMatrix, distCoeffs, rvec, tvec);
		}
		imshow("out", imageCopy);
		char key = (char)waitKey(waitTime);
		if(key == 27) {
			break;
		}

		double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
		totalTime += currentTime;
		totalIterations++;
		if(totalIterations % 30 == 0) {
			cout << "Detection Time = " << currentTime * 1000 << " ms "
					<< "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
		}

		ros::spinOnce();
		loop_rate.sleep();
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
	ROS_INFO("Ending Session...\n");
	ros::shutdown();

    return 0;
}


