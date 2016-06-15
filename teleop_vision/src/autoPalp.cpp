/*
 * calib_robot_board.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: nearlab
 */


#include "autoPalp.hpp"

using namespace std;
using namespace cv;

/**
 */
boardDetector::boardDetector(int _markersX,int _markersY, float _markerLength,
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





rosObj::rosObj(int argc, char *argv[], string n_name){
	freq_ros = 0;
	all_good = false;
	camId_param = 0;
	node_name = n_name;
}


/**
 */
int main(int argc, char *argv[]) {



    cvNamedWindow("AutoPalp", CV_WINDOW_NORMAL);
    cvSetWindowProperty("AutoPalp", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);



	std::string node_name("calib_robot");
	ros::init(argc, argv, node_name);
	rosObj r(argc, argv, node_name);
	r.init();
//	if(!r.all_good) return 1;
	string robot_topic_name;


	//-----------------------------------------------------------------------------------
	// ros initialization
	//-----------------------------------------------------------------------------------
	ros::Rate loop_rate(r.freq_ros);
    ros::Duration d(0.5);
//	string robot_topic_name_param = "/FKCartCurrent";
//	string force_topic_name_param = "/FKCartCurrent";

    cout << "r.force_topic_name_param  "  << r.force_topic_name_param << endl;
    cout << "r.robot_topic_name_param  "  << r.robot_topic_name_param << endl;
	ros::Subscriber sub_robot = r.n.subscribe(r.robot_topic_name_param, 10, &rosObj::robotPoseCallback, &r);
	ros::Subscriber sub_force = r.n.subscribe(r.force_topic_name_param, 10, &rosObj::forceCallback, &r);
	ros::Publisher pub_br_pose = r.n.advertise<geometry_msgs::Pose>("board_to_robot_pose",1,0);
	ros::Publisher pub_bc_pose = r.n.advertise<geometry_msgs::Pose>("board_to_cam_pose",1,0);

    int status = 2;
    const Scalar RED(0,0,255), GREEN(0,255,0), CYAN(255,255,0), ORANGE(35,64,255);
	vector<Point3d> calibPoints3d;
    vector<Point2d> calibPoints2d;
	geometry_msgs::Pose br_pose_msg;
	geometry_msgs::Pose bc_pose_msg;
	Mat imageCopy;
	Vec3d bc_rvec, bc_tvec, br_tvec;
	KDL::Frame bc_frame, br_frame;
	vector<double> br_vec(7, 0.0);

    Matx33d  br_rotm= Matx33d::zeros();
	double m_to_px = r.markerLength_px_param/r.markerlength_m_param;

	//-----------------------------------------------------------------------------------
	// Set up the camera
	//-----------------------------------------------------------------------------------
	Mat camMatrix, distCoeffs;
    string cam_data_path = "/home/nima/workspace/arucotest/trust_camera_data.xml";

    int waitTime= 1;
    bool readOk = boardDetector::readCameraParameters(cam_data_path, camMatrix, distCoeffs);
    if(!readOk) {
    	cerr << "Invalid camera file" << endl;
    	return 0;
    }
    VideoCapture inputVideo;
    inputVideo.open(r.camId_param);

    //-----------------------------------------------------------------------------------
    // Construct the board detector object
    //-----------------------------------------------------------------------------------
    if(ros::param::has(node_name+"/board_to_robot_tr")){
    	vectorToPoseMsg(r.board_to_robot_tr_param, br_pose_msg);
    	tf::poseMsgToKDL(br_pose_msg, br_frame);
    	kdlRotToMatx33d(br_frame.M, br_rotm);
    	cout<< "Set the board_to_robot_tr parameter: " << r.board_to_robot_tr_param << endl;
    	ROS_INFO("Initialization done.");
    }

    //-----------------------------------------------------------------------------------
    // Construct the board detector object
    //-----------------------------------------------------------------------------------
    boardDetector b(r.markersX_param, r.markersY_param,  r.markerLength_px_param,
			 r.markerSeparation_px_param,  r.dictionaryId_param, camMatrix, distCoeffs);

    //-----------------------------------------------------------------------------------
    // Construct the calibBoardRobot object
    //-----------------------------------------------------------------------------------
    calibBoardRobot cbr;
	string msg = "";
    // make 3 points at the corner of the boards for calibration
    calibPoints3d.push_back(Point3f(0, 0, 0));
    calibPoints3d.push_back(Point3f(b.markersX*b.markerLength + (b.markersX-1)* b.markerSeparation, 0, 0));
    calibPoints3d.push_back(Point3f(0, b.markersY*b.markerLength + (b.markersY-1)* b.markerSeparation, 0));

    //-----------------------------------------------------------------------------------
    // Construct the Palpation object
    //-----------------------------------------------------------------------------------
    palpMap pm;
	vector<Point2d> palpPoints2d;

    while(ros::ok() && inputVideo.isOpened()) {

    	//-----------------------------------------------------------------------------------
    	// DETECT BOARD
    	//-----------------------------------------------------------------------------------
    	inputVideo >> b.image;
    	b.detect(bc_rvec, bc_tvec);
    	b.drawAxis();
    	rvectvecToKdlFrame(bc_rvec, bc_tvec, bc_frame);
    	//		b.drawDetectedMarkers();
    	// draw results
    	b.image.copyTo(imageCopy);
//
//    	if(b.markersOfBoardDetected > 0){
//    		drawAC(imageCopy, camMatrix, distCoeffs, bc_rvec, bc_tvec);
//    	}

    	msg = ("press k to calibrate or p to start palpation.");
        char key = (char)waitKey(waitTime);
        if(key == 27)
        	break;
        else if(key == 107){
        	status = 0;
        	cbr.reset();
        }
        else if(key == 112){

        	status = (status==1) ? 2:1;

    		//reset the palpation map
        	if(status==1){
        		pm.reset();
        		pm.palp_done = false;
        	}
        }

    	//-----------------------------------------------------------------------------------
    	// calibration if needed
    	//-----------------------------------------------------------------------------------
        if(status == 0){

        	// project the calibration points to show it in the image
        	if(b.markersOfBoardDetected > 0){
        		projectPoints(calibPoints3d, bc_rvec, bc_tvec, camMatrix, distCoeffs, calibPoints2d);

        		cbr.setVisualPoints(calibPoints2d);

        		cbr.updateMe(msg, imageCopy);
        	}

        	if(key == 32){

        		//space pressed save the position of the point
        		cbr.setCalibpoint(r.robotPose.position.x,r.robotPose.position.y,r.robotPose.position.z);

        		// check the state of calibration
        		if(cbr.updateMe(msg, imageCopy)){

        			// calibration is done
        			status = 2;

        			// get the calibration data
        			cbr.get_tr(br_rotm,br_tvec);

        			// convert the calibration data
        			matx33dToKdlRot(br_rotm, br_frame.M);
        			br_frame.p.data[0] = br_tvec.val[0];
        			br_frame.p.data[1] = br_tvec.val[1];
        			br_frame.p.data[2] = br_tvec.val[2];

        			tf::poseKDLToMsg(br_frame, br_pose_msg);
        			poseMsgToVector(br_pose_msg, br_vec);
        			r.n.setParam(node_name+"/board_to_robot_tr", br_vec);
        			cout<< "Set the board_to_robot_tr parameter: " << br_vec << endl;
        			pub_br_pose.publish(br_pose_msg);
        		}
        	}


        }
        else if(status==1){
        	//-----------------------------------------------------------------------------------
        	// Palpation mode
        	//-----------------------------------------------------------------------------------

        	// just consider 1 out of three runs, to reduce the number of generate points and memory load
        	if(pm.counter <0){
        		pm.counter++;
        	}
        	else{
        		pm.counter = 0;
        		Point3d br_trans = Point3d( br_frame.p[0],  br_frame.p[1],  br_frame.p[2]);
        		// save current position of the tool
        		Point3d toolPoint3d_rrf = Point3d(r.robotPose.position.x , r.robotPose.position.y, r.robotPose.position.z);

        		// taking the robot tool tip from the robot ref frame to board ref frame and convert to pixles from meters
        		Point3d temp = br_rotm.t() * ( toolPoint3d_rrf - br_trans);
        		Point3d toolPoint3d_crf = Point3d(temp.x* m_to_px, temp.y* m_to_px, temp.z* m_to_px) ;

        		// register current point and force
        		pm.registerPoint(toolPoint3d_crf, r.msrd_force);
        		pm.palp_done = true;

        	}
    		msg = "In automatic palpation mode. Press p to terminate";

        }
        else{

        	//-----------------------------------------------------------------------------------
        	// Drawing the tool tip
        	//-----------------------------------------------------------------------------------
        	Point3d br_trans = Point3d( br_frame.p[0],  br_frame.p[1],  br_frame.p[2]);
        	Point3d toolPoint3d_rrf = Point3d(r.robotPose.position.x , r.robotPose.position.y, r.robotPose.position.z);

        	// taking the robot tool tip from the robot ref frame to board ref frame and convert to pixles from meters
        	Point3d temp = br_rotm.t() * ( toolPoint3d_rrf - br_trans);
        	Point3d toolPoint3d_crf = Point3d(temp.x* m_to_px, temp.y* m_to_px, temp.z* m_to_px) ;

        	vector<Point3d> toolPoint3d_vec_crf;
        	vector<Point2d> toolPoint2d;
        	toolPoint3d_vec_crf.push_back(toolPoint3d_crf);

        	projectPoints(toolPoint3d_vec_crf, bc_rvec, bc_tvec, camMatrix, distCoeffs, toolPoint2d);
        	circle( imageCopy, toolPoint2d[0], 4, ORANGE, 2);

        	//-----------------------------------------------------------------------------------
        	// publish board to camera pos
        	//-----------------------------------------------------------------------------------
        	tf::poseKDLToMsg(bc_frame, bc_pose_msg);
        	pub_bc_pose.publish(bc_pose_msg);

        }

    	//-----------------------------------------------------------------------------------
    	// draw the palpation map
    	//-----------------------------------------------------------------------------------
        if(pm.palp_done){


        	projectPoints(pm.getCoordinates(), bc_rvec, bc_tvec, camMatrix, distCoeffs, palpPoints2d);

        	for(int i=0; i<(pm.getCoordinates().size()); i++){
            	circle( imageCopy, palpPoints2d[i], 4, pm.getColors()[i], -1);

        	}
        }

    	//-----------------------------------------------------------------------------------
    	// Output Text
    	//-----------------------------------------------------------------------------------
    	int baseLine = 0;
//    	Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
//    	Point textOrigin(imageCopy.cols - 2*textSize.width - 10, imageCopy.rows - 2*baseLine - 10);
    	Point textOrigin(10,10);
    	putText( imageCopy, msg, textOrigin, 1, 1, (status == 0) ?   RED:GREEN);

        imshow("AutoPalp", imageCopy);

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

void kdlRotToMatx33d(const KDL::Rotation _kdl,  cv::Matx33d &_mat ){
	_mat = cv::Matx33d(_kdl(0,0),_kdl(0,1),_kdl(0,2),_kdl(1,0),_kdl(1,1),_kdl(1,2),_kdl(2,0),_kdl(2,1),_kdl(2,2));
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
void vectorToPoseMsg(const vector<double> in_vec, geometry_msgs::Pose& out_pose){

	out_pose.position.x = in_vec.at(0);
	out_pose.position.y = in_vec.at(1);
	out_pose.position.z = in_vec.at(2);
	out_pose.orientation.x = in_vec.at(3);
	out_pose.orientation.y = in_vec.at(4);
	out_pose.orientation.z = in_vec.at(5);
	out_pose.orientation.w = in_vec.at(6);

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
    double w = 100;
    double h = 100;
    double d = 200;
    double x_start = 960;
    double y_start = 600;
    double z_start = 0;
    axisPoints.push_back(Point3f(x_start,   y_start+h, z_start));
    axisPoints.push_back(Point3f(x_start+w, y_start+h, z_start));
    axisPoints.push_back(Point3f(x_start+w, y_start,   z_start));
    axisPoints.push_back(Point3f(x_start,   y_start,   z_start));

    axisPoints.push_back(Point3f(x_start,   y_start+h, z_start+d));
    axisPoints.push_back(Point3f(x_start+w, y_start+h, z_start+d));
    axisPoints.push_back(Point3f(x_start+w, y_start,   z_start+d));
    axisPoints.push_back(Point3f(x_start,   y_start,   z_start+d));

    vector< Point2f > imagePoints;
    projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

    // draw axis lines
    int points[7] = {0,1,2,4,5,6};
    for(int i= 0; i<6; i++){
        line(_image, imagePoints[points[i]], imagePoints[points[i]+1], Scalar(200, 100, 10), 3);
    }
    for(int i= 0; i<4; i++){
        line(_image, imagePoints[i], imagePoints[i+4], Scalar(200, 100, 10), 3);
    }
    line(_image, imagePoints[3], imagePoints[0], Scalar(200, 100, 10), 3);
    line(_image, imagePoints[7], imagePoints[4], Scalar(200, 100, 10), 3);
}

ostream& operator<<(ostream& out, const vector<double>& vect){
	for (unsigned int iter = 0; iter < vect.size(); ++iter) {
		out << "[" << iter <<"]: "<<vect.at(iter) << "\t";
	}
	//	cout << endl;

	//    out << "{ id = " << m.id << ", ";
	//    out << "X = " << m.X << ", ";
	//    out << "A = " << m.A << "}";
	return out;
}



//-----------------------------------------------------------------------------------
// ROS OBJECT CLASS METHODS
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// init
//-----------------------------------------------------------------------------------

void rosObj::init(){
	all_good = true;
	n.param<double>("frequency", freq_ros, 25);

	if(!ros::param::has(node_name+"/cam_data_path"))  {
		ROS_ERROR("Parameter cam_data_path is required.");
		all_good = false;
	}
	else n.getParam(node_name+"/cam_data_path", cam_data_path_param);

	if(!ros::param::has(node_name+"/robot_topic_name")){
		ROS_INFO("Parameter robot_topic_name not found. Setting /FKCartCurrent");
		robot_topic_name_param = "/FKCartCurrent";
	}
	else n.getParam(node_name+"/robot_topic_name", robot_topic_name_param);

	if(!ros::param::has(node_name+"/force_topic_name")){
		ROS_INFO("Parameter force_topic_name not found. Setting /force");
		force_topic_name_param = "/force";
	}
	else n.getParam(node_name+"/force_topic_name", force_topic_name_param);

	if(!ros::param::has(node_name+"/board_to_robot_tr")){
		ROS_INFO("Parameter board_to_robot_tr is not set. Calibration is required.");
//		all_good = false;
	}
	else n.getParam(node_name+"/board_to_robot_tr", board_to_robot_tr_param);


	n.param<int>(node_name+"/markersX", markersX_param, 9);
	n.param<int>(node_name+"/markersY", markersY_param, 6);
	n.param<float>(node_name+"/markerLength_px", markerLength_px_param, 100);
	n.param<float>(node_name+"/markerSeparation_px", markerSeparation_px_param, 20);
	n.param<int>(node_name+"/dictionaryId", dictionaryId_param, 9);
	n.param<float>(node_name+"/markerlength_m", markerlength_m_param, 0.027);
	n.param<int>(node_name+"/camId", camId_param, 0);

}


//-----------------------------------------------------------------------------------
// robotPoseCallback
//-----------------------------------------------------------------------------------

void rosObj::robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	//    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
	robotPose.position = msg->position;
	robotPose.orientation = msg->orientation;
//		ros::Duration d(0.01);
//		d.sleep();
}


//-----------------------------------------------------------------------------------
// forceCallback
//-----------------------------------------------------------------------------------

void rosObj::forceCallback(const geometry_msgs::Wrench::ConstPtr& msg)
{
	//    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
	msrd_force.force = msg->force;

//		ros::Duration d(0.01);
//		d.sleep();
}


//-----------------------------------------------------------------------------------
// PALPATION MAP CLASS METHODS
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// registerPoint
//-----------------------------------------------------------------------------------

void palpMap::registerPoint(const Point3d _coord, const geometry_msgs::Wrench _wrench){

	//Save the coordinates
	coordinates.push_back(_coord);

	//generate a color based on the z axis force
	double fz = _wrench.force.z;
	if(fz<0)
		fz=0;
//	else
//		fz-=4;

	double f_max = 5;
	if(fz > f_max)
		fz = f_max;
	colors.push_back(Scalar( (1-fz/f_max)*255, 20, (fz/f_max)*255 ));

}
//-----------------------------------------------------------------------------------
// reset. remove saved coordinates and colors
//-----------------------------------------------------------------------------------
void palpMap::reset(){

	while(coordinates.size()>0){
		coordinates.pop_back();
		colors.pop_back();
	}

}

//-----------------------------------------------------------------------------------
// BOARD DETECtOR CLASS METHODS
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// detect
//-----------------------------------------------------------------------------------

void boardDetector::detect(Vec3d& _rvec, Vec3d& _tvec){
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

//-----------------------------------------------------------------------------------
// drawAxis
//-----------------------------------------------------------------------------------
void boardDetector::drawAxis(){
	if(markersOfBoardDetected > 0)
		aruco::drawAxis(image, camMatrix, distCoeffs, rvec, tvec, axisLength);
}

//-----------------------------------------------------------------------------------
// drawDetectedMarkers
//-----------------------------------------------------------------------------------
void boardDetector::drawDetectedMarkers(){
	if(ids.size() > 0)
		aruco::drawDetectedMarkers(image, corners, ids);
}



//-----------------------------------------------------------------------------------
// readCameraParameters
//-----------------------------------------------------------------------------------
bool boardDetector::readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


//-----------------------------------------------------------------------------------
// CALIB BOARD ROBOT CLASS METHODS
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// updateMe
//-----------------------------------------------------------------------------------
bool calibBoardRobot::updateMe(string & msg, cv::Mat  img){


	int pointsGot = axisPoints.size();
	circle( img, visual_calib_points[pointsGot], 5, Scalar(255,255,0), 4);
	if(pointsGot < 3){
		// Need more points. show the message for the next point
		std::ostringstream oss;
		message = "Point at point ";
		oss << pointsGot + 1;
		message += oss.str();
	}
	msg = message;

	return calib_status;
}

//-----------------------------------------------------------------------------------
// setCalibpoint
//-----------------------------------------------------------------------------------
void calibBoardRobot::setCalibpoint(double x, double y, double z){

	axisPoints.push_back(Point3f( x, y, z));
	cout << "got: " << x << " "<< y << " "<< z << endl;

	int pointsGot = axisPoints.size();

	if(pointsGot ==3){

		// got all the 3 points, find the transformation
		make_tr(axisPoints, calib_rotm, calib_tvec);

		// set the status as done
		calib_status = true;
		message = "Calibration Done.";
	}



}

//-----------------------------------------------------------------------------------
// make_tr
//-----------------------------------------------------------------------------------
void calibBoardRobot::make_tr(vector< Point3d > axisPoints, Matx33d & _rotm, Vec3d & br_tvec){

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

	cout << "Rotation:" <<endl;
	cout <<  _rotm(0,0) << " "<< _rotm(0,1) << " "<< _rotm(0,2) << endl;
	cout <<  _rotm(1,0) << " "<< _rotm(1,1) << " "<< _rotm(1,2) << endl;
	cout <<  _rotm(2,0) << " "<< _rotm(2,1) << " "<< _rotm(2,2) << endl;
	cout << "Translation:" <<endl;
	cout << br_tvec[0] << endl;
	cout << br_tvec[1] << endl;
	cout << br_tvec[2] << endl;
//	br_rotm.push_back(yy );
//	br_rotm.push_back(zz );
//	br_rotm.t();


//	cout << "x: " << x.val[0] << " "<< x.val[1] << " "<< x.val[2] << endl;
//	cout << "y: " << y.val[0] << " "<< y.val[1] << " "<< y.val[2] << endl;
//	cout << "z: " << z.val[0] << " "<< z.val[1] << " "<< z.val[2] << endl;

}

void calibBoardRobot::get_tr(Matx33d & _rotm, Vec3d & _br_tvec){
	_rotm = calib_rotm;
	_br_tvec = calib_tvec;

}

void calibBoardRobot::reset(){
	cout << " initial size of axis points = " << axisPoints.size() << endl;
	calib_status = false;

	while(axisPoints.size()>0){
		axisPoints.pop_back();
	}
}



