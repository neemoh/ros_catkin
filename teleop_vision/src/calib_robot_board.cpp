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
#include "kdl/frames.hpp"
#include "geometry_msgs/Pose.h"
#include <tf_conversions/tf_kdl.h>
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


class rosObj {
public:
	rosObj(int argc, char *argv[], string n_name){
		freq_ros = 0;
		all_good = false;
		camId_param = 0;
		node_name = n_name;
	}
	void init(){
		all_good = true;
		n.param<double>("frequency", freq_ros, 25);

		if(!ros::param::has(node_name+"/cam_data_path"))  {
			ROS_ERROR("Parameter cam_data_path is required.");
			all_good = false;
		}
		else n.getParam(node_name+"/cam_data_path", cam_data_path_param);

		if(!ros::param::has(node_name+"/robot_topic_name")){
			ROS_ERROR("Parameter robot_topic_name is required.");
			all_good = false;
		}
		else n.getParam(node_name+"/robot_topic_name", robot_topic_name_param);

		n.param<int>(node_name+"/markersX", markersX_param, 9);
		n.param<int>(node_name+"/markersY", markersY_param, 6);
		n.param<float>(node_name+"/markerLength_px", markerLength_px_param, 100);
		n.param<float>(node_name+"/markerSeparation_px", markerSeparation_px_param, 20);
		n.param<int>(node_name+"/dictionaryId", dictionaryId_param, 9);
		n.param<float>(node_name+"/markerlength_m", markerlength_m_param, 0.027);
		n.param<int>(node_name+"/camId", camId_param, 0);


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
	std::string node_name;
	ros::NodeHandle n;
	double freq_ros;
	int camId_param;
//	ros::Subscriber sub_robot;
	geometry_msgs::Pose robotPose;
	int markersX_param;
	int markersY_param;
	float markerLength_px_param;
	float markerSeparation_px_param;
	int dictionaryId_param;
	float markerlength_m_param;
};

void rvecTokdlRot(const cv::Vec3d _rvec, KDL::Rotation & _kdl);
void rvectvecToKdlFrame(const cv::Vec3d _rvec,const cv::Vec3d _tvec, KDL::Frame & _kdl);
void matx33dToKdlRot(const cv::Matx33d _mat, KDL::Rotation & _kdl );
void poseMsgToVector(const geometry_msgs::Pose in_pose, vector<double>& out_vec);
void make_tr(vector< Point3d > axisPoints, Matx33d & _rotm, Vec3d & br_tvec);
void drawAC(InputOutputArray _image, InputArray _cameraMatrix, InputArray _distCoeffs,
              InputArray _rvec, InputArray _tvec);
/**
 */
int main(int argc, char *argv[]) {

	std::string node_name("calib_robot");
	ros::init(argc, argv, node_name);
	rosObj r(argc, argv, node_name);
	r.init();
//	if(!r.all_good) return 1;
	string robot_topic_name;

    //-----------------------------------------------------  ros initialization

	ros::Rate loop_rate(r.freq_ros);
//	ros::Subscriber sub_robot = r.n.subscribe(r.robot_topic_name_param, 10, &rosObj::robotPoseCallback, &r);
	string robot_topic_name_param = "/FKCartCurrent";
	ros::Subscriber sub_robot = r.n.subscribe(robot_topic_name_param, 10, &rosObj::robotPoseCallback, &r);
	ros::Publisher pub_br_pose = r.n.advertise<geometry_msgs::Pose>("board_to_robot_pose",1,0);
	ros::Publisher pub_bc_pose = r.n.advertise<geometry_msgs::Pose>("board_to_cam_pose",1,0);

    int status = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0), CYAN(255,255,0), YELLOW(25,210,255);
    vector<Point3d> axisPoints;
	vector<Point3d> calibPoints3d;
    vector<Point2d> calibPoints2d;
	geometry_msgs::Pose br_pose_msg;
	geometry_msgs::Pose bc_pose_msg;
    Matx33d  br_rotm= Matx33d::zeros();
	double m_to_px = r.markerLength_px_param/r.markerlength_m_param;

    //-------------------------------    Set up the camera  -----------------------------
    Mat camMatrix, distCoeffs;
    string cam_data_path = "/home/nima/workspace/arucotest/trust_camera_data.xml";

    int waitTime= 1;
//    bool readOk = readCameraParameters(r.cam_data_path_param, camMatrix, distCoeffs);
    bool readOk = readCameraParameters(cam_data_path, camMatrix, distCoeffs);
    if(!readOk) {
    	cerr << "Invalid camera file" << endl;
    	return 0;
    }
    VideoCapture inputVideo;
    inputVideo.open(r.camId_param);
//    double totalTime = 0;
//    int totalIterations = 0;

    boardDetector b(r.markersX_param, r.markersY_param,  r.markerLength_px_param,
			 r.markerSeparation_px_param,  r.dictionaryId_param, camMatrix, distCoeffs);

    calibPoints3d.push_back(Point3f(0, 0, 0));
    calibPoints3d.push_back(Point3f(b.markersX*b.markerLength + (b.markersX-1)* b.markerSeparation, 0, 0));
    calibPoints3d.push_back(Point3f(0, b.markersY*b.markerLength + (b.markersY-1)* b.markerSeparation, 0));
    ros::Duration d(0.5);

	Mat imageCopy;
	Vec3d bc_rvec, bc_tvec, br_tvec;
	KDL::Frame bc_frame, br_frame;


    ROS_INFO("Initialization done.");
    while(ros::ok() && inputVideo.isOpened()) {


        //----------------------------- DETECT BOARD ------------------------------------------------------------------------------
    	inputVideo >> b.image;
    	b.detect(bc_rvec, bc_tvec);
    	b.drawAxis();
    	rvectvecToKdlFrame(bc_rvec, bc_tvec, bc_frame);
    	//		b.drawDetectedMarkers();
    	// draw results
    	b.image.copyTo(imageCopy);

    	if(b.markersOfBoardDetected > 0){
    		drawAC(imageCopy, camMatrix, distCoeffs, bc_rvec, bc_tvec);
    		projectPoints(calibPoints3d, bc_rvec, bc_tvec, camMatrix, distCoeffs, calibPoints2d);
    	}


        //----------------------------- Output Text ------------------------------------------------------------------------------
        //! [output_text]
		string msg = (status == 1) ? "Point at point 1" :
					 (status == 2) ? "Point at point 2" :
					 (status == 3) ? "Point at point 3" :
					 (status == 5) ? "Calibration done. Esc to exit." : "Press 'space' to start";

        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(imageCopy.cols - 2*textSize.width - 10, imageCopy.rows - 2*baseLine - 10);

        putText( imageCopy, msg, textOrigin, 1, 1, (status == 0 || status == 5) ?  GREEN : RED);

        //----------------------------- record calibration points ------------------------------------------------------------------------------
        char key = (char)waitKey(waitTime);
        if(key == 27) {
        	break;
        }else if(key == 32 && status <4){
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

        //----------------------------- show calibration points ------------------------------------------------------------------------------
		vector<double> br_vec(7, 0.0);

        switch (status){
		case 1:
		    circle( imageCopy, calibPoints2d[0], 5,   CYAN, 4);
        	break;
		case 2:
			circle( imageCopy, calibPoints2d[1], 5,   CYAN, 4);
			break;
		case 3:
			circle( imageCopy, calibPoints2d[2], 5,   CYAN, 4);
			break;

		case 4:
			// Hgot the points, find the transform and set it as a parameter
			make_tr(axisPoints, br_rotm, br_tvec);
			matx33dToKdlRot(br_rotm, br_frame.M);
			br_frame.p.data[0] = br_tvec.val[0];
			br_frame.p.data[1] = br_tvec.val[1];
			br_frame.p.data[2] = br_tvec.val[2];

			cout << "Rotation:" <<endl;
			cout <<  br_rotm(0,0) << " "<< br_rotm(0,1) << " "<< br_rotm(0,2) << endl;
			cout <<  br_rotm(1,0) << " "<< br_rotm(1,1) << " "<< br_rotm(1,2) << endl;
			cout <<  br_rotm(2,0) << " "<< br_rotm(2,1) << " "<< br_rotm(2,2) << endl;
			cout << "Translation:" <<endl;
			cout << axisPoints[0].x << endl;
			cout << axisPoints[0].y << endl;
			cout << axisPoints[0].z << endl;

			tf::poseKDLToMsg(br_frame, br_pose_msg);

			poseMsgToVector(br_pose_msg, br_vec);
			r.n.setParam(node_name+"/board_to_robot_tr", br_vec);
			cout<< "Set the board_to_robot_tr parameter: " << br_vec[0] <<" "<<br_vec[1] <<" "<<br_vec[2] <<" "<<
					br_vec[3] <<" "<<br_vec[4] <<" "<<br_vec[5] <<" "<<br_vec[6] << endl;

			pub_br_pose.publish(br_pose_msg);
			status++;
			break;

		case 5 :
			//-------------------------------- Drawing the tool tip ---------------------------------------------------------------
			Point3d br_trans = Point3d( br_frame.p[0],  br_frame.p[1],  br_frame.p[2]);
			Point3d toolPoint3d_rrf = Point3d(r.robotPose.position.x , r.robotPose.position.y, r.robotPose.position.z);

			// taking the robot tool tip from the robot ref frame to board ref frame and convert to pixles from meters
			Point3d temp = br_rotm.t() * ( toolPoint3d_rrf - br_trans);
			Point3d toolPoint3d_crf = Point3d(temp.x* m_to_px, temp.y* m_to_px, temp.z* m_to_px) ;

			vector<Point3d> toolPoint3d_vec_crf;
			vector<Point2d> toolPoint2d;
			toolPoint3d_vec_crf.push_back(toolPoint3d_crf);

			projectPoints(toolPoint3d_vec_crf, bc_rvec, bc_tvec, camMatrix, distCoeffs, toolPoint2d);
			circle( imageCopy, toolPoint2d[0], 4, YELLOW, 2);

			//-------------------------------- publish board to camera pose --------------------------------------------------------
			tf::poseKDLToMsg(bc_frame, bc_pose_msg);
			pub_bc_pose.publish(bc_pose_msg);
			break;
        }
        imshow("out", imageCopy);

        ros::spinOnce();
        loop_rate.sleep();
	}

	ROS_INFO("Ending Session...\n");
	ros::shutdown();

    return 0;
}

// ---------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------          FUNCTIONS         -----------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------
void rvecTokdlRot(const cv::Vec3d _rvec, KDL::Rotation & _kdl){
	cv::Matx33d	mat;
	cv::Rodrigues(_rvec, mat);
	_kdl = KDL::Rotation(mat(0,0),mat(0,1),mat(0,2),mat(1,0),mat(1,1),mat(1,2),mat(2,0),mat(2,1),mat(2,2));
}

void rvectvecToKdlFrame(const cv::Vec3d _rvec,const cv::Vec3d _tvec, KDL::Frame & _kdl){
	cv::Matx33d	mat;
	cv::Rodrigues(_rvec, mat);
	_kdl.M = KDL::Rotation(mat(0,0),mat(0,1),mat(0,2),mat(1,0),mat(1,1),mat(1,2),mat(2,0),mat(2,1),mat(2,2));
	_kdl.p.data[0]= _tvec.val[0];
	_kdl.p.data[1]= _tvec.val[1];
	_kdl.p.data[2]= _tvec.val[2];
}

void matx33dToKdlRot(const cv::Matx33d _mat, KDL::Rotation & _kdl ){
	_kdl = KDL::Rotation(_mat(0,0),_mat(0,1),_mat(0,2),_mat(1,0),_mat(1,1),_mat(1,2),_mat(2,0),_mat(2,1),_mat(2,2));

}

void poseMsgToVector(const geometry_msgs::Pose in_pose, vector<double>& out_vec) {

	out_vec.at(0) = in_pose.position.x;
	out_vec.at(1) = in_pose.position.y;
	out_vec.at(2) = in_pose.position.z;
	out_vec.at(3) = in_pose.orientation.x;
	out_vec.at(4) = in_pose.orientation.y;
	out_vec.at(5) = in_pose.orientation.z;
	out_vec.at(6) = in_pose.orientation.w;
}
void make_tr(vector< Point3d > axisPoints, Matx33d & _rotm, Vec3d & br_tvec){

	br_tvec = axisPoints[0];

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
	Matx33d br_rotm = Matx33d::ones();

	_rotm(0,0) = x[0]; _rotm(0,1) = y[0]; _rotm(0,2) = z[0];
	_rotm(1,0) = x[1]; _rotm(1,1) = y[1]; _rotm(1,2) = z[1];
	_rotm(2,0) = x[2]; _rotm(2,1) = y[2]; _rotm(2,2) = z[2];

//	br_rotm.push_back(yy );
//	br_rotm.push_back(zz );
//	br_rotm.t();


//	cout << "x: " << x.val[0] << " "<< x.val[1] << " "<< x.val[2] << endl;
//	cout << "y: " << y.val[0] << " "<< y.val[1] << " "<< y.val[2] << endl;
//	cout << "z: " << z.val[0] << " "<< z.val[1] << " "<< z.val[2] << endl;

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
