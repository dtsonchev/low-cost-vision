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

visionNode::visionNode(int argc, char* argv[]){
		if (argc != 4){
			printUsage(argv[0]);
			exit(1);
		}
		int device_number = atoi(argv[1]);
		int format_number = atoi(argv[2]);
		cam = new unicap_cv_camera(device_number,format_number);
		cam->set_auto_white_balance(true);
		cam->set_exposure(0.015);
		camFrame = Mat(cam->get_img_height(), cam->get_img_width(), cam->get_img_format());

		fidDetector = new FiducialDetector();
		fidDetector->minRad = 15;
		fidDetector->maxRad = 25;
		fidDetector->minDist = 2.0f;
		fidDetector->maxDist = 5.0f;

		qrDetector = new QRCodeDetector();

		point2f::point2fvector rc;
		rc.push_back(point2f(62, 108));
		rc.push_back(point2f(-64, 112));
		rc.push_back(point2f(-66, -76.5));
		cordTransformer= new pc_rc_transformer(rc,rc);

		rectifier = new RectifyImage();
		if(!rectifier->initRectify(argv[3], cv::Size( cam->get_img_width(),cam->get_img_height()))){
			cout << "XML not found" << endl;
			exit(2);
		}
		crateMovementThresshold = 5;
		crateTracker = new CrateTracker(10 , crateMovementThresshold);

		invokeCalibration = false;

		crateEventPublisher = node.advertise<vision::CrateEventMsg>("crateEvent", 100);
		ErrorPublisher = node.advertise<vision::error>("visionError", 100);
		getCrateService = node.advertiseService("getCrate", &visionNode::getCrate, this);
		getAllCratesService = node.advertiseService("getAllCrates", &visionNode::getAllCrates, this);
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
		markers.push_back(point2f(medianX(fid1_buffer), medianY(fid1_buffer)));
		markers.push_back(point2f(medianX(fid2_buffer), medianY(fid2_buffer)));
		markers.push_back(point2f(medianX(fid3_buffer), medianY(fid3_buffer)));
		cordTransformer->set_fiducials_pixel_coordinates(markers);

		// Determine mean deviation
		double totalDistance = 0;
		for(std::vector<cv::Point2f>::iterator it=fid1_buffer.begin(); it!=fid1_buffer.end(); ++it) totalDistance += Crate::dist(markers[0], *it);
		for(std::vector<cv::Point2f>::iterator it=fid2_buffer.begin(); it!=fid2_buffer.end(); ++it) totalDistance += Crate::dist(markers[1], *it);
		for(std::vector<cv::Point2f>::iterator it=fid3_buffer.begin(); it!=fid3_buffer.end(); ++it) totalDistance += Crate::dist(markers[2], *it);
		float meanDeviation = totalDistance / double(fid1_buffer.size()+fid2_buffer.size()+fid3_buffer.size());

		ROS_INFO("Calibration markers updated. Measured: %d Failed: %d Deviation: %f", measurements, failCount, meanDeviation);
		return true;
	}
	ROS_INFO("Calibration timed out, too many failed attempts. Measurements needed: %d Measured: %d", measurements, measurementCount);
	return false;
}

void visionNode::run(){
	if(!calibrate()) ros::shutdown();

	while(ros::ok()){
		if(invokeCalibration) {
			invokeCalibration = false;
			calibrate();
		}

		cam->get_frame(&camFrame);
		rectifier->rectify(camFrame, rectifiedCamFrame);
		cv::Mat gray;
		cv::cvtColor(rectifiedCamFrame, gray, CV_BGR2GRAY);

		for(point2f::point2fvector::iterator it=markers.begin(); it!=markers.end(); ++it)
			cv::circle(rectifiedCamFrame, cv::Point(cv::saturate_cast<int>(it->x), cv::saturate_cast<int>(it->y)), 1, cv::Scalar(0, 0, 255), 2);

		std::vector<Crate> crates;
		qrDetector->detectCrates(gray, crates);
		for(std::vector<Crate>::iterator it=crates.begin(); it!=crates.end(); ++it)
		{
			it->draw(rectifiedCamFrame);


			std::vector<cv::Point2f> points;
			for(int n = 0; n <3; n++){
				point2f result = cordTransformer->to_rc(point2f(it->getPoints()[n].x, it->getPoints()[n].y));
				points.push_back(cv::Point2f(result.x, result.y));
			}

			//transfor coordinates
			it->setPoints(points);

			//ROS_INFO("crate pos: %f %f", it->rect().center.x, it->rect().center.y);
		}
		std::vector<CrateEvent> events = crateTracker->update(crates);

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

		imshow("image",rectifiedCamFrame);
		waitKey(10);
		ros::spinOnce();
	}
}
