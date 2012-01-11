#pragma once
#include <pcrctransformation/pcrctransformer.hpp>
#include <pcrctransformation/point2f.hpp>
#include <CameraCalibration/RectifyImage.h>
#include <unicap_cv_bridge.hpp>
#include <FiducialDetector.h>
#include <QRCodeDetector.h>
#include <Crate.h>
#include <vision/crateTracker.h>
#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

#include <vision/CrateEventMsg.h>
#include <vision/error.h>
#include <vision/getCrate.h>
#include <vision/getAllCrates.h>

class visionNode{
public:
	visionNode(int argc, char* argv[]);
	~visionNode();

	void run();
	bool getCrate(vision::getCrate::Request &req,vision::getCrate::Request &res);
	bool getAllCrates(vision::getAllCrates::Request &req,vision::getAllCrates::Request &res);

private:
	unicap_cv_bridge::unicap_cv_camera * cam;
	FiducialDetector * fidDetector;
	QRCodeDetector * qrDetector;
	pcrctransformation::pc_rc_transformer * cordTransformer;
	RectifyImage * rectifier;
	cv::Mat camFrame;
	cv::Mat rectifiedCamFrame;

	ros::NodeHandle node;
	ros::Publisher crateEventPublisher;
	ros::Publisher ErrorPublisher;

	double crateMovementThresshold;
	int numberOfStableFrames;

	void printUsage(char* invokeName);
};
