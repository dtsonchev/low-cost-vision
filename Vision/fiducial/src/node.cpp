/*
 * node.cpp
 *
 *  Created on: Dec 21, 2011
 *      Author: Jules Blok
 */

#include <FiducialDetector/FiducialDetector.h>
#include <imageMetaData/Types.hpp>
#include <imageMetaData/Tools.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <ctime>
#include <cmath>
#include <boost/foreach.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fiducial/fiducial_cfg.h>
#include <fiducial/fiducial_feedback.h>

#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

inline float dist(cv::Point2f p1, cv::Point2f p2) { return(sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))); }

FiducialDetector detector;

void commandHandler(const fiducial::fiducial_cfg::ConstPtr& msg){
	detector.minRad = msg->minRad;
	detector.maxRad = msg->maxRad;
	detector.distance = msg->distance;
	detector.circleVotes = msg->circleVotes;
	detector.lineVotes = msg->lineVotes;
	detector.minDist = msg->minDist;
	detector.maxDist = msg->maxDist;
	detector.lowThreshold = msg->lowThreshold;
	detector.highThreshold = msg->highThreshold;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "fiducial_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("fiducial_cfg", 1000, commandHandler);
	ros::Publisher pub = n.advertise<fiducial::fiducial_feedback>("fiducial_feedback", 1000);

    if (argc < 3) {
            std::cout << "Usage: " << argv[0] << " <xml path> <xml root tag>\n" << std::endl;
            return 1;
    }

    // Load the metadata of the images from an XML file
    std::vector<imageMetaData::ImageMD> images = imageMetaData::getMetaData(argv[1], argv[2]);

    srand((int)time(NULL)); // Seed random number generated

    char key = -1;
	while(key != 'q') {
		imageMetaData::ImageMD& img = images[rand() % images.size()];
		fiducial::fiducial_feedback feedback;

		cv::Mat image = cv::imread(img.path);
		if(!image.data) {
			std::cout << "Image " << img.path << " not found! Aborting test..." << std::endl;
			return 1;
		}
		cv::Mat debug = image.clone();

		feedback.imagePath = img.path;
		feedback.tooFewLines = 0;
		feedback.outsideROI = 0;

		cv::Mat gray;
		cv::cvtColor(image, gray, CV_BGR2GRAY);

		std::vector<cv::Point2f> points;
		detector.detect(gray, points, &feedback, &debug);
		feedback.fiducialCount = points.size();

		std::vector<cv::Point2f> targetPoints;
		foreach(imageMetaData::Properties object, img.objects) {
			targetPoints.push_back(cv::Point2f((int)object["fid1_x"], (int)object["fid1_y"]));
			targetPoints.push_back(cv::Point2f((int)object["fid2_x"], (int)object["fid2_y"]));
			targetPoints.push_back(cv::Point2f((int)object["fid3_x"], (int)object["fid3_y"]));
		}
		feedback.expectedFiducialCount = targetPoints.size();

		float totalDistance = 0;
		foreach(cv::Point2f target, targetPoints) {
			float lastDistance = dist(points[0],target);
			foreach(cv::Point2f point, points) {
				float d = dist(point,target);
				if(d < lastDistance) {
					lastDistance = d;
				}
			}
			totalDistance += lastDistance;
		}
		feedback.avgDeviation = totalDistance / (float)targetPoints.size();

		pub.publish(feedback);

		cv::imshow("Debug", debug);

		ros::spinOnce();

		key = cv::waitKey(1000);
	}

	return 0;
}
