//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        VisionNode
// File:           visionNode.cpp
// Description:    his vision RosNode detects the crates and publishes events on the crateEvent topic.
// Author:         Kasper van Nieuwland en Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of VisionNode.
//
// VisionNode is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// VisionNode is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with VisionNode.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#include <vision/visionNode.h>
#include <pcrctransformation/pcrctransformer.hpp>
#include <pcrctransformation/point2f.hpp>
#include <CameraCalibration/RectifyImage.h>
#include <unicap_cv_bridge.hpp>
#include <FiducialDetector.h>
#include <QRCodeDetector.h>
#include <Crate.h>
#include <ros/ros.h>

#include <vision/CrateEventMsg.h>
#include <vision/error.h>
#include <vision/getCrate.h>
#include <vision/getAllCrates.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <map>

using namespace unicap_cv_bridge;
using namespace pcrctransformation;
using namespace std;
using namespace cv;

//on mouse click event, print the real life coordinate at the clicked pixel
void on_mouse(int event, int x, int y, int flags, void* param){
	if(event == CV_EVENT_LBUTTONDOWN){
		pc_rc_transformer* cordTransformer = (pc_rc_transformer*)param;
		point2f result = cordTransformer->to_rc(point2f(x, y));
		ROS_INFO("X: %f, Y:%f", result.x, result.y);
	}
}

visionNode::visionNode(int argc, char* argv[]){
		if (argc != 4){
			printUsage(argv[0]);
			exit(1);
		}
		//setup the camera
		int device_number = atoi(argv[1]);
		int format_number = atoi(argv[2]);
		cam = new unicap_cv_camera(device_number,format_number);
		cam->set_auto_white_balance(true);
		cam->set_exposure(0.015);
		camFrame = Mat(cam->get_img_height(), cam->get_img_width(), cam->get_img_format());

		//setup the fiducial detector
		fidDetector = new FiducialDetector();
		fidDetector->minRad = 15;
		fidDetector->maxRad = 25;
		fidDetector->minDist = 2.0f;
		fidDetector->maxDist = 5.0f;

		//setup the QR detector
		qrDetector = new QRCodeDetector();

		//setup the coordinate transformation(from pixel to real life)
		//The real-life coordinates of the fiducials in mm.
		//0,0 is center of the delta robot
		point2f::point2fvector rc;
		rc.push_back(point2f(61.5 + 1.5, 110.5));
		rc.push_back(point2f(-62.5 + 1.5, 113.5));
		rc.push_back(point2f(-65 + 1.5, -74));
		cordTransformer= new pc_rc_transformer(rc,rc);

		//crate tracking configuration
		//the amount of mm a point has to move before we mark it as moving.
		crateMovementThresshold = 0.25;
		//the number of frames before a change is marked definite.
		numberOfStableFrames = 10;
		crateTracker = new CrateTracker(numberOfStableFrames , crateMovementThresshold);

		//setup the camera lens distortion corrector
		rectifier = new RectifyImage();
		if(!rectifier->initRectify(argv[3], cv::Size( cam->get_img_width(),cam->get_img_height()))){
			cout << "XML not found" << endl;
			exit(2);
		}

		invokeCalibration = false;

		//ROS things
		crateEventPublisher = node.advertise<vision::CrateEventMsg>("crateEvent", 100);
		ErrorPublisher = node.advertise<vision::error>("visionError", 100);
		getCrateService = node.advertiseService("getCrate", &visionNode::getCrate, this);
		getAllCratesService = node.advertiseService("getAllCrates", &visionNode::getAllCrates, this);

		//GUI stuff
		cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
		cvSetMouseCallback("image", &on_mouse, cordTransformer);
}

bool visionNode::getCrate(vision::getCrate::Request &req,vision::getCrate::Response &res)
{
	exCrate crate;
	bool succeeded = crateTracker->getCrate(req.name, crate);
	if(succeeded){
		res.state = crate.getState();
		vision::CrateMsg msg;
		msg.name = crate.name;
		msg.x =crate.rect().center.x;
		msg.y =crate.rect().center.y;
		msg.angle = crate.rect().angle;
		res.crate = msg;
	} else{
		res.state = exCrate::state_non_existing;
		vision::CrateMsg msg;
		msg.name = "";
		msg.x =0;
		msg.y =0;
		msg.angle = 0;
		res.crate = msg;
	}
	return true;
}

bool visionNode::getAllCrates(vision::getAllCrates::Request &req,vision::getAllCrates::Response &res)
{
	std::vector<exCrate> allCrates = crateTracker->getAllCrates();
	for(std::vector<exCrate>::iterator it = allCrates.begin(); it != allCrates.end(); ++it)
	{
		res.states.push_back(it->getState());
		vision::CrateMsg msg;
		msg.name =it->name;
		msg.x =it->rect().center.x;
		msg.y =it->rect().center.y;
		msg.angle = it->rect().angle;
		res.crates.push_back(msg);
	}
	return true;
}

bool visionNode::recalibrate(std_srvs::Empty &req, std_srvs::Empty &res){
	invokeCalibration = true;
	return true;
}

visionNode::~visionNode(){
	delete cam;
	delete fidDetector;
	delete qrDetector;
	delete cordTransformer;
	delete rectifier;
	delete crateTracker;
}

void visionNode::printUsage(char* invokeName){
	printf("usage for starting capture: %s device_number format_number correction xml\n", invokeName);
}

bool xComp(cv::Point2f i, cv::Point2f j) { return (i.x<j.x); }
bool yComp(cv::Point2f i, cv::Point2f j) { return (i.y<j.y); }

inline float medianX(std::vector<cv::Point2f> points){
       	std::vector<cv::Point2f>::iterator n = points.begin()+points.size()/2;
	if(points.size()%2 == 0) {
		nth_element(points.begin(), n, points.end(), xComp);
		return ((n->x + (n+1)->x)/2.0);
	} else {
	        nth_element(points.begin(), n, points.end(), xComp);
        	return n->x;
	}
}
inline float medianY(std::vector<cv::Point2f> points){
        std::vector<cv::Point2f>::iterator n = points.begin()+points.size()/2;
        if(points.size()%2 == 0) {
                nth_element(points.begin(), n, points.end(), yComp);
                return ((n->y + (n+1)->y)/2.0);
        } else {
                nth_element(points.begin(), n, points.end(), yComp);
                return n->y;
        }
}

bool visionNode::calibrate(unsigned int measurements, int maxErrors){
	ROS_INFO("Updating calibration markers...");

	std::vector<cv::Point2f> fid1_buffer;
    std::vector<cv::Point2f> fid2_buffer;
    std::vector<cv::Point2f> fid3_buffer;

	unsigned int measurementCount = 0;
	unsigned int failCount = 0;
	while(measurementCount<measurements && (maxErrors<0 || failCount<maxErrors)){
		cam->get_frame(&camFrame);
		rectifier->rectify(camFrame, rectifiedCamFrame);
		cv::Mat gray;
		cv::cvtColor(rectifiedCamFrame, gray, CV_BGR2GRAY);

		std::vector<cv::Point2f> fiducialPoints;
		fidDetector->detect(gray, fiducialPoints);
		if(fiducialPoints.size() == 3) {
			measurementCount++;
			Crate::order(fiducialPoints);
			fid1_buffer.push_back(fiducialPoints[0]);
			fid2_buffer.push_back(fiducialPoints[1]);
			fid3_buffer.push_back(fiducialPoints[2]);
		} else {
			failCount++;
			ROS_WARN("Incorrect number of markers. Needed: 3 Saw: %d", fiducialPoints.size());
		}
	}

	if(measurementCount == measurements) {
		cv::Point2f fid1(medianX(fid1_buffer), medianY(fid1_buffer));
		cv::Point2f fid2(medianX(fid2_buffer), medianY(fid2_buffer));
		cv::Point2f fid3(medianX(fid3_buffer), medianY(fid3_buffer));

		markers.push_back(point2f(fid1.x, fid1.y));
		markers.push_back(point2f(fid2.x, fid2.y));
		markers.push_back(point2f(fid3.x, fid3.y));
		cordTransformer->set_fiducials_pixel_coordinates(markers);

		// Determine mean deviation
		double totalDistance = 0;
		for(std::vector<cv::Point2f>::iterator it=fid1_buffer.begin(); it!=fid1_buffer.end(); ++it) totalDistance += Crate::distance(fid1, *it);
		for(std::vector<cv::Point2f>::iterator it=fid2_buffer.begin(); it!=fid2_buffer.end(); ++it) totalDistance += Crate::distance(fid2, *it);
		for(std::vector<cv::Point2f>::iterator it=fid3_buffer.begin(); it!=fid3_buffer.end(); ++it) totalDistance += Crate::distance(fid3, *it);
		float meanDeviation = totalDistance / double(fid1_buffer.size()+fid2_buffer.size()+fid3_buffer.size());

		ROS_INFO("Calibration markers updated.\nMeasured: %d Failed: %d Mean deviation: %f", measurements, failCount, meanDeviation);
		return true;
	}
	ROS_INFO("Calibration timed out, too many failed attempts. Measurements needed: %d Measured: %d", measurements, measurementCount);
	return false;
}

void visionNode::run(){
	//run initial calibration. If that fails, this node will shut down.
	if(!calibrate()) ros::shutdown();
	
	VideoWriter outputVideo;
	Size S = cv::Size(cam->get_img_width(),cam->get_img_height());
	outputVideo.open("/home/lcv/output.avi" , CV_FOURCC('M','P','2','V'), 30, S, true);

	//main loop
	while(ros::ok()){

		//if calibration was manualy invoked by call on the service
		if(invokeCalibration) {
			invokeCalibration = false;
			calibrate();
		}

		//grab frame from camera
		cam->get_frame(&camFrame);

		//correct the lens distortion
		rectifier->rectify(camFrame, rectifiedCamFrame);

		//create a duplicate grayscale frame
		cv::Mat gray;
		cv::cvtColor(rectifiedCamFrame, gray, CV_BGR2GRAY);

		//draw the calibration points
		for(point2f::point2fvector::iterator it=markers.begin(); it!=markers.end(); ++it)
			cv::circle(rectifiedCamFrame, cv::Point(cv::saturate_cast<int>(it->x), cv::saturate_cast<int>(it->y)), 1, cv::Scalar(0, 0, 255), 2);

		//detect crates
		std::vector<Crate> crates;
		qrDetector->detectCrates(gray, crates);

		//transform crate coordinates
		for(std::vector<Crate>::iterator it=crates.begin(); it!=crates.end(); ++it)
		{
			it->draw(rectifiedCamFrame);

			std::vector<cv::Point2f> points;
			for(int n = 0; n <3; n++){
				point2f result = cordTransformer->to_rc(point2f(it->getPoints()[n].x, it->getPoints()[n].y));
				points.push_back(cv::Point2f(result.x, result.y));
			}
			it->setPoints(points);
		}

		//inform the crate tracker about the seen crates
		std::vector<CrateEvent> events = crateTracker->update(crates);

		//publish events
		for(std::vector<CrateEvent>::iterator it = events.begin(); it != events.end(); ++it)
		{
			vision::CrateEventMsg msg;
			msg.event = it->type;
			msg.crate.name = it->name;
			msg.crate.x = it->x;
			msg.crate.y = it->y;
			msg.crate.angle = it->angle;

			ROS_INFO(it->toString().c_str());
			crateEventPublisher.publish(msg);
		}

		//update GUI
		outputVideo.write(rectifiedCamFrame);
		imshow("image",rectifiedCamFrame);
		waitKey(1000/30);

		//let ROS do it's magical things
		ros::spinOnce();
	}
}
