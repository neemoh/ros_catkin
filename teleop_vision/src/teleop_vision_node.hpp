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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>


using namespace std;
using namespace cv;




//-----------------------------------------------------------------------------------
// BOARD DETECtOR CLASS
//-----------------------------------------------------------------------------------

class boardDetector{
public:
	boardDetector(
			int _markersX,
			int _markersY,
			float _markerLength,
			float _markerSeparation,
			int _dictionaryId,
			Mat& _camMatrix,
			Mat& _distCoeffs,
			double _n_avg);

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
	bool ready;

	// number of averaging points to prevent board frame oscillation
	double n_avg;
	unsigned int init_counter;

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
	void toolCurrCallback(const geometry_msgs::Pose::ConstPtr& msg);
	void toolDestCallback(const geometry_msgs::Pose::ConstPtr& msg);

public:
	bool all_good;
	std::string cam_data_path_param;
	std::string tool_curr_topic_name_param;
	std::string tool_dest_topic_name_param;
	std::string node_name;
	ros::NodeHandle n;
	double freq_ros;
	int camId_param;
//	ros::Subscriber sub_robot;
	geometry_msgs::Pose tool_curr_pose;
	geometry_msgs::Pose tool_dest_pose;
	sensor_msgs::PointCloud2 ac_point_cloud;
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

	bool updateMe(string & msg, cv::Mat img);

	void setCalibpoint(double x, double y, double z);

	void make_tr(vector< Point3d > axisPoints, Matx33d & _rotm, Vec3d & br_tvec);

	void get_tr(Matx33d & _rotm, Vec3d & br_tvec);

	void setVisualPoints(vector<Point2d> in) {visual_calib_points = in;};

	void reset();
private:
    vector<Point3d> axisPoints;
    bool calib_status;
    string message;
    Vec3d calib_tvec;
    Matx33d calib_rotm;
    vector<Point2d> visual_calib_points;
};




//-----------------------------------------------------------------------------------
// AC CLASS
//-----------------------------------------------------------------------------------

class drawings{

public:
	drawings(
			const Mat _camMatrix,
			const Mat _distCoeffs,
			const KDL::Frame _br_frame,
			const double m_to_px);

//	ac_square(Point3d _center, Point2d _dims) {	center = _center;
//	dims = _dims;};

	void setSquare(Point3d _center, Point2d _dims) {	sq_center = _center;
		sq_dims = _dims;};


	// drawing a simple rectangle
	void create3dCurve(double _center_x, double _center_y, unsigned int n_points);
	void createCircle(double _center_x, double _center_y, double a, double b, unsigned int n_points);

	void drawSquare(InputOutputArray _image);
	void drawToolTip(InputOutputArray _image, double _x, double _y, double _z, const Scalar _color);

	void drawGuidingLines(InputOutputArray _image, const geometry_msgs::Pose & tool, const geometry_msgs::Pose & desired);

	void drawACPoints(InputOutputArray _image);
	void update_cam_2_board_ref(Vec3d _bc_rvec,	Vec3d _bc_tvec){
		bc_rvec = _bc_rvec; bc_tvec = _bc_tvec;
	}

	void getACCloud(sensor_msgs::PointCloud2 & cloud_out);

public:

	Mat camMatrix;
	Mat distCoeffs;

	Vec3d bc_rvec;
	Vec3d bc_tvec;

//	KDL::Frame br_frame;
	Point3d br_trans;
	Matx33d br_rotm;

	double m_to_px;
	// draw the AC
	Point3d sq_center;
	Point2d sq_dims;
    vector< Point3d > sq_points_in_board;
    vector< Point3d > ac_points_in_board;
    vector< Point3d > ac_points_in_robot;

};

namespace conversions {

// some self explanetory conversions!
void rvecTokdlRot(const cv::Vec3d _rvec, KDL::Rotation & _kdl);

void rvectvecToKdlFrame(const cv::Vec3d _rvec,const cv::Vec3d _tvec, KDL::Frame & _kdl);

void matx33dToKdlRot(const cv::Matx33d _mat, KDL::Rotation & _kdl );

void kdlRotToMatx33d(const KDL::Rotation _kdl,  cv::Matx33d &_mat );

void poseMsgToVector(const geometry_msgs::Pose in_pose, vector<double>& out_vec);

void vectorToPoseMsg(const vector<double> in_vec, geometry_msgs::Pose& out_pose);


};

//-----------------------------------------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------------------------------------

// drawing a simple box frame
void drawCube(InputOutputArray _image, InputArray _cameraMatrix, InputArray _distCoeffs,
              InputArray _rvec, InputArray _tvec);


// operator overload to print out vectors
ostream& operator<<(ostream& out, const vector<double>& vect);


#endif /* TELEOP_VISION_SRC_BOARD_AC_HPP_ */
