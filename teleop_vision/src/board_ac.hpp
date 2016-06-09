/*
 * calib_board_robot.hpp
 *
 *  Created on: Jun 9, 2016
 *      Author: nima
 */

#ifndef TELEOP_VISION_SRC_BOARD_AC_HPP_
#define TELEOP_VISION_SRC_BOARD_AC_HPP_

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


//-----------------------------------------------------------------------------------
// BOARD DETECtOR CLASS
//-----------------------------------------------------------------------------------

class boardDetector{
public:
	boardDetector(int _markersX,int _markersY, float _markerLength,
			float _markerSeparation, int _dictionaryId, Mat& _camMatrix,
			Mat& _distCoeffs );

	void detect(Vec3d& _rvec, Vec3d& _tvec);

	void drawAxis();

	void drawDetectedMarkers();

	static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);


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



//-----------------------------------------------------------------------------------
// ROSOBJ CLASS
//-----------------------------------------------------------------------------------

class rosObj {
public:

	rosObj(int argc, char *argv[], string n_name);
	void init();
	void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

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
	vector<double> board_to_robot_tr_param;
	int markersX_param;
	int markersY_param;
	float markerLength_px_param;
	float markerSeparation_px_param;
	int dictionaryId_param;
	float markerlength_m_param;
};




//-----------------------------------------------------------------------------------
// CALIBBOARDROBOT CLASS
//-----------------------------------------------------------------------------------
class calibBoardRobot {

public:

	calibBoardRobot() {
		calib_status=0;
		message = "Point at point 1 and press 'space' to start calibration";
	};

	bool updateMe(string & msg, cv::Mat& img);

	void setCalibpoint(double x, double y, double z);

	void make_tr(vector< Point3d > axisPoints, Matx33d & _rotm, Vec3d & br_tvec);

	void get_tr(Matx33d & _rotm, Vec3d & br_tvec);

	void setVisualPoints(vector<Point2d> in) {visual_calib_points = in;};

private:
    vector<Point3d> axisPoints;
	vector<Point3d> calibPoints3d;
    vector<Point2d> calibPoints2d;
    bool calib_status;
    string message;
    Vec3d calib_tvec;
    Matx33d calib_rotm;
    vector<Point2d> visual_calib_points;
};


//-----------------------------------------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------------------------------------




#endif /* TELEOP_VISION_SRC_BOARD_AC_HPP_ */
