//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Fiducial
// File:           main.cpp
// Description:    Main function for testing purposes
// Author:         Jules Blok
// Notes:          None
//
// License:        GNU GPL v3
//
// This file is part of Fiducial.
//
// Fiducial is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Fiducial is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Fiducial.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include "FiducialDetector.h"
#include "CrateDetector.h"
#include "QRCodeDetector.h"
#include "Crate.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

FiducialDetector fidDetector;
CrateDetector crateDetector;
QRCodeDetector qrDetector;

bool showValues = true;
bool showContours = false;
bool showDebug = true;

void processFid(cv::Mat& image, cv::Mat& debug) {
	std::vector<cv::Point2f> points;
	cv::RotatedRect rect;
	if(showDebug)
		fidDetector.detect(image, points, &debug);
	else
		fidDetector.detect(image, points);

	if(showContours) {
		cv::Mat canny;
		cv::Canny(image, canny, fidDetector.lowThreshold, fidDetector.highThreshold);
		cv::imshow("Canny", canny);
		cv::Canny(image, canny, crateDetector.lowThreshold, crateDetector.highThreshold);
		std::vector<std::vector<cv::Point> > contoursCanny;
		cv::findContours(canny, contoursCanny, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cv::Mat cannyContours = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

		for(unsigned int i=0; i<contoursCanny.size(); i++)
			cv::drawContours(cannyContours, contoursCanny, i, cv::Scalar(rand()%255,rand()%255,rand()%255), 2);

		if(showValues) {
			cv::rectangle(cannyContours, cv::Point(10, 5), cv::Point(210, 22), cv::Scalar(100, 100, 100, 50), CV_FILLED, 1, 0);
			std::stringstream ss;
			ss << "Threshold: " << crateDetector.lowThreshold << "/" << crateDetector.highThreshold;
			cv::putText(cannyContours, ss.str(), cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		}

		cv::imshow("Contours", cannyContours);
	}

	if(points.size() > 3) {
		std::vector<Crate> crates;
		crateDetector.detect(image, crates, points);

		if(showDebug) for(std::vector<Crate>::iterator it=crates.begin(); it!=crates.end(); ++it) it->draw(debug);
	}
	else if(points.size() == 3) {
		CrateDetector::order(points);
		Crate crate(points);
		if(showDebug) crate.draw(debug);
	}

	if(showValues) {
		cv::rectangle(debug, cv::Point(10, 5), cv::Point(210, 110), cv::Scalar(100, 100, 100, 50), CV_FILLED, 1, 0);
		std::stringstream ss;
		ss << "Votes: " << fidDetector.lineVotes << "/" << fidDetector.maxLines << " | " << fidDetector.circleVotes;
		cv::putText(debug, ss.str(), cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		ss.str("");
		ss << "Circle radius: " << fidDetector.minRad << "/" << fidDetector.maxRad;
		cv::putText(debug, ss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		ss.str("");
		ss << "Line distance: " << fidDetector.minDist << "/" << fidDetector.maxDist;
		cv::putText(debug, ss.str(), cv::Point(20, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		ss.str("");
		ss << "Threshold: " << fidDetector.lowThreshold << "/" << fidDetector.highThreshold;
		cv::putText(debug, ss.str(), cv::Point(20, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		ss.str("");
		ss << "Method: " << ((fidDetector.centerMethod==FiducialDetector::MEAN)?"Mean":(fidDetector.centerMethod==FiducialDetector::MEDOID_RHO)?"MedoidRho":"MedoidTheta");
		cv::putText(debug, ss.str(), cv::Point(20, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
	}
}

void processQR(cv::Mat& image, cv::Mat& debug) {
	std::vector<Crate> crates;

	qrDetector.detectCrates(image, crates);

	if(showDebug) for(std::vector<Crate>::iterator it=crates.begin(); it!=crates.end(); ++it) it->draw(debug);
}

void callback(char key, cv::Mat* image = NULL) {
	switch(key)
	{
		case 'w': fidDetector.lineVotes++; break;
		case 's': fidDetector.lineVotes--; break;
		case 'a': fidDetector.circleVotes--; break;
		case 'd': fidDetector.circleVotes++; break;
		case ']': fidDetector.maxLines++; break;
		case '[': fidDetector.maxLines--; break;
		case 'u': fidDetector.minRad++; break;
		case 'j': fidDetector.minRad--; break;
		case 'h': fidDetector.maxRad--; break;
		case 'k': fidDetector.maxRad++; break;
		case 'r': fidDetector.minDist+=.1; break;
		case 'f': fidDetector.minDist-=.1; break;
		case 't': fidDetector.maxDist+=.1; break;
		case 'g': fidDetector.maxDist-=.1; break;
		case 'x': fidDetector.lowThreshold+=10; break;
		case 'z': fidDetector.lowThreshold-=10; break;
		case 'v': fidDetector.highThreshold+=10; break;
		case 'c': fidDetector.highThreshold-=10; break;
		case 'm': crateDetector.lowThreshold+=10; break;
		case 'n': crateDetector.lowThreshold-=10; break;
		case '.': crateDetector.highThreshold+=10; break;
		case ',': crateDetector.highThreshold-=10; break;
		case 'b': showValues=!showValues; break;
		case '/': showContours=!showContours; break;
		case 'p': showDebug=!showDebug; break;
		case 'y': imwrite("screenshot.png", *image); break;
		case '\\': fidDetector.centerMethod++; fidDetector.centerMethod%=3; break;
	}
}

void usage(char* cmd) {
	std::cout << "Usage:\t" << cmd << " image <path>" << std::endl;
	std::cout << "\t" << cmd << " cam <index>" << std::endl;
}

int main(int argc, char* argv[]) {
	if(argc < 2) {
		usage(argv[0]);
		return 1;
	}

	fidDetector.verbose = true;

	if(!strcmp(argv[1], "image")) {
		if(argc < 3) {
			usage(argv[0]);
			return 1;
		}

		cv::Mat image = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR); // Read the file
		if (!image.data) // Check for invalid input
		{
			std::cout << "Could not open or find the image" << std::endl;
			return 1;
		}

		cv::Mat gray;
		cv::cvtColor(image, gray, CV_BGR2GRAY);

		// Retrieve camera frames
		char key = 0;
		while (key != 'q') {
			cv::Mat debug = image.clone();
			processQR(gray, debug);

			cv::imshow("Image", debug);
			key = cv::waitKey();
			callback(key, &image);
		}
	}
	else if(!strcmp(argv[1], "cam")) {
		if(argc < 3) {
			usage(argv[0]);
			return 1;
		}

		// Initialize camera
		cv::Mat frame;
		cv::VideoCapture cam(atoi(argv[2]));

		if (!cam.isOpened()) {
			std::cout << "Failed to open camera" << std::endl;
			return -1;
		}

		// Retrieve camera frames
		char key = 0;
		while (key != 'q') {
			cam.read(frame);

			cv::Mat gray;
			cv::cvtColor(frame, gray, CV_BGR2GRAY);

			processQR(gray, frame);

			cv::imshow("Frame", frame);
			key = cv::waitKey(10);
			callback(key, &frame);
		}
	}

	return 0;
}
