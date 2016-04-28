/*
 * calib_robot_board.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: nearlab
 */




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
	boardDetector(int _markersX,int _markersY, float _markerLength,
			float _markerSeparation, int _dictionaryId, Mat& _camMatrix,
			Mat& _distCoeffs ){

		markersX = _markersX;
		markersY = _markersY;
		markerLength = _markerLength;
		markerSeparation = _markerSeparation;
		dictionaryId = _dictionaryId;
		camMatrix = _camMatrix;
		distCoeffs = _distCoeffs;
        axisLength = 200;
		refindStrategy = false;
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




void make_tr(vector< Point3f > axisPoints, Matx33d & _calib_rotm){

	Vec3f x = Vec3f(axisPoints[1].x - axisPoints[0].x,
						axisPoints[1].y - axisPoints[0].y,
						axisPoints[1].z - axisPoints[0].z);

	Vec3f y = Vec3f(axisPoints[2].x - axisPoints[0].x,
						axisPoints[2].y - axisPoints[0].y,
						axisPoints[2].z - axisPoints[0].z);
//	Vec3f x = Vec3f(-2.0, -0.2, 0.02);
//	Vec3f y = Vec3f(-0.2, -2.0, 0.04);
	cv::normalize(x,x);
	cv::normalize(y,y);
	Vec3f z =  x.cross(y);
	cv::normalize(z,z);
	x =  y.cross(z);
	y =  z.cross(x);
	Matx33d calib_rotm = Matx33d::ones();

	calib_rotm(0,0) = x[0]; calib_rotm(0,1) = y[0]; calib_rotm(0,2) = z[0];
	calib_rotm(1,0) = x[1]; calib_rotm(1,1) = y[1]; calib_rotm(1,2) = z[1];
	calib_rotm(2,0) = x[2]; calib_rotm(2,1) = y[2]; calib_rotm(2,2) = z[2];

	_calib_rotm = calib_rotm;
//	calib_rotm.push_back(yy );
//	calib_rotm.push_back(zz );
//	calib_rotm.t();
cout << "finished 1" << endl;


//	cout << "x: " << x.val[0] << " "<< x.val[1] << " "<< x.val[2] << endl;
//	cout << "y: " << y.val[0] << " "<< y.val[1] << " "<< y.val[2] << endl;
//	cout << "z: " << z.val[0] << " "<< z.val[1] << " "<< z.val[2] << endl;

}
class rosObj {
public:
	rosObj(int argc, char *argv[]){
		freq_ros = 0;
		all_good = false;
		camId = 0;
	}
	void init(){
		all_good = true;
		n.param<double>("frequency", freq_ros, 30);

		if(!ros::param::has("calib_robot/cam_data_path"))  {
			ROS_ERROR("Parameter cam_data_path is required.");
			all_good = false;
		}
		else n.getParam("calib_robot/cam_data_path", cam_data_path_param);

		if(!ros::param::has("calib_robot/robot_topic_name")){
			ROS_ERROR("Parameter robot_topic_name is required.");
			all_good = false;
		}
		else n.getParam("calib_robot/robot_topic_name", robot_topic_name_param);

		n.param<int>("calib_robot/markersX", markersX_param, 9);
		n.param<int>("calib_robot/markersY", markersY_param, 6);
		n.param<float>("calib_robot/markerLength_px", markerLength_px_param, 100);
		n.param<float>("calib_robot/markerSeparation_px", markerSeparation_px_param, 20);
		n.param<int>("calib_robot/dictionaryId", dictionaryId_param, 9);
		n.param<float>("calib_robot/markerlength_m", markerlength_m_param, 0.027);

	}
	void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
	{
		//    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
		robotPose.position = msg->position;
		robotPose.orientation = msg->orientation;
//		ros::Duration d(0.01);
//		d.sleep();
	}
public:
	bool all_good;
	std::string cam_data_path_param;
	std::string robot_topic_name_param;
	ros::NodeHandle n;
	double freq_ros;
	int camId;
//	ros::Subscriber sub_robot;
	geometry_msgs::Pose robotPose;
	int markersX_param;
	int markersY_param;
	float markerLength_px_param;
	float markerSeparation_px_param;
	int dictionaryId_param;
	float markerlength_m_param;
};

/**
 */
int main(int argc, char *argv[]) {
	ros::init(argc, argv, "ACboard");
	rosObj r(argc, argv);
	r.init();
	if(!r.all_good) return 1;
	string robot_topic_name;


    //-----------------------------------------------------  ros initialization

	ros::Rate loop_rate(r.freq_ros);
	ros::Subscriber sub_robot = r.n.subscribe(r.robot_topic_name_param, 10, &rosObj::robotPoseCallback, &r);

    int status = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0), CYAN(255,255,0);
    vector< Point3f > axisPoints,calibPoints3d;
    vector<Point2f> calibPoints2d;
    Matx33d  calib_rotm= Matx33d::zeros();
	double m_to_px = r.markerLength_px_param/r.markerlength_m_param;

    //-------------------------------    Set up the camera  -----------------------------
    int camId = 1;
    Mat camMatrix, distCoeffs;
//    string cam_data_path = "/home/nearlab/workspace/cameracalib/Debug/out_camera_data.xml";

    int waitTime= 1;
    bool readOk = readCameraParameters(r.cam_data_path_param, camMatrix, distCoeffs);
    if(!readOk) {
    	cerr << "Invalid camera file" << endl;
    	return 0;
    }
    VideoCapture inputVideo;
    inputVideo.open(r.camId);

    double totalTime = 0;
    int totalIterations = 0;

    boardDetector b(r.markersX_param, r.markersY_param,  r.markerLength_px_param,
			 r.markerSeparation_px_param,  r.dictionaryId_param, camMatrix, distCoeffs);

    calibPoints3d.push_back(Point3f(0, 0, 0));
    calibPoints3d.push_back(Point3f(b.markersX*b.markerLength + (b.markersX-1)* b.markerSeparation, 0, 0));
    calibPoints3d.push_back(Point3f(0, b.markersY*b.markerLength + (b.markersY-1)* b.markerSeparation, 0));

    ROS_INFO("Initialization done.");

    while(ros::ok() && inputVideo.isOpened()) {

    	Mat imageCopy;
    	Vec3d rvec, tvec;

    	inputVideo >> b.image;
		b.detect(rvec, tvec);
		b.drawAxis();
//		b.drawDetectedMarkers();
		// draw results
		b.image.copyTo(imageCopy);

		if(b.markersOfBoardDetected > 0){
			drawAC(imageCopy, camMatrix, distCoeffs, rvec, tvec);
		    projectPoints(calibPoints3d, rvec, tvec, camMatrix, distCoeffs, calibPoints2d);
		}


        //----------------------------- Output Text ------------------------------------------------
        //! [output_text]
		string msg = (status == 1) ? "Point at point 1" :
					 (status == 2) ? "Point at point 2" :
					 (status == 3) ? "Point at point 3" : "Press 'space' to start";

        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(imageCopy.cols - 2*textSize.width - 10, imageCopy.rows - 2*baseLine - 10);

        putText( imageCopy, msg, textOrigin, 1, 1, status == 0 ?  GREEN : RED);

        char key = (char)waitKey(waitTime);
        if(key == 27) {
        	break;
        }else if(key == 32){
        	if (status==1){
        	    axisPoints.push_back(Point3f(r.robotPose.position.x,r.robotPose.position.y,r.robotPose.position.z));
        		cout << "got: " << r.robotPose.position.x << " "<< r.robotPose.position.y << " "<< r.robotPose.position.z << endl;
        	}else if (status==2){
        	    axisPoints.push_back(Point3f(r.robotPose.position.x,r.robotPose.position.y,r.robotPose.position.z));
        		cout << "got: " << r.robotPose.position.x << " "<< r.robotPose.position.y << " "<< r.robotPose.position.z << endl;
        	}else if(status == 3){
        	    axisPoints.push_back(Point3f(r.robotPose.position.x,r.robotPose.position.y,r.robotPose.position.z));
        		cout << "got: " << r.robotPose.position.x << " "<< r.robotPose.position.y << " "<< r.robotPose.position.z << endl;
        	}
        	status++;
        }

        switch (status){
		case 1:
		    circle( imageCopy, calibPoints2d[0], 8,   CYAN, 5);
        	break;
		case 2:
		    circle( imageCopy, calibPoints2d[1], 8,   CYAN, 5);
        	break;
		case 3:
		    circle( imageCopy, calibPoints2d[2], 8,   CYAN, 5);
        	break;
        }
        if(status==4){
        	make_tr(axisPoints, calib_rotm);
        	cout << "Rotation:" <<endl;
        	cout <<  calib_rotm(0,0) << " "<< calib_rotm(0,1) << " "<< calib_rotm(0,2) << endl;
        	cout <<  calib_rotm(1,0) << " "<< calib_rotm(1,1) << " "<< calib_rotm(1,2) << endl;
        	cout <<  calib_rotm(2,0) << " "<< calib_rotm(2,1) << " "<< calib_rotm(2,2) << endl;
        	cout << "Translation:" <<endl;
        	cout << axisPoints[0].x << endl;
        	cout << axisPoints[0].y << endl;
        	cout << axisPoints[0].z << endl;

        	status++;
        }
        else if(status==5){

        	Point3d calib_translation = Point3d(r.robotPose.position.x *m_to_px, r.robotPose.position.y *m_to_px, r.robotPose.position.z *m_to_px);
        	Point3d toolPoint3d_crf; //cam ref frame
        	Point3d toolPoint3d_rrf; //robot ref frame
        	toolPoint3d_crf = calib_rotm.t() * (toolPoint3d_rrf - calib_translation);

        	vector<Point3d> toolPoint3d_vec_crf; //cam ref frame
        	vector<Point2d> toolPoint2d;
        	toolPoint3d_vec_crf.push_back(toolPoint3d_crf);

        	projectPoints(toolPoint3d_vec_crf, rvec, tvec, camMatrix, distCoeffs, toolPoint2d);
		    circle( imageCopy, toolPoint2d[0], 8,   GREEN, 5);
        }
        imshow("out", imageCopy);

        ros::spinOnce();
        loop_rate.sleep();
	}



	ROS_INFO("Ending Session...\n");
	ros::shutdown();

    return 0;
}


