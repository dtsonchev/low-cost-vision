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
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
QRCodeDetector qrDetector;

bool showValues = true;
bool showContours = false;
bool showDebug = true;

void process(cv::Mat& image, cv::Mat& debug) {
	std::vector<cv::Point2f> points;

	if(showDebug)
		fidDetector.detect(image, points, &debug);
	else
		fidDetector.detect(image, points);

	if(showContours) {
		// Apply gaussian blur
		cv::Mat blur;
		cv::GaussianBlur(image, blur, cv::Size(fidDetector.blur,fidDetector.blur), fidDetector.sigma);
		cv::imshow("Blur", blur);

		cv::Mat canny;
		cv::Canny(blur, canny, fidDetector.lowThreshold, fidDetector.highThreshold);
		cv::imshow("Canny", canny);
	}

	std::vector<Crate> crates;
	qrDetector.detectCrates(image, crates);
	if(showDebug) for(std::vector<Crate>::iterator it=crates.begin(); it!=crates.end(); ++it) it->draw(debug);

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
		ss << "Blur: " << fidDetector.blur << "/" << fidDetector.sigma;
		cv::putText(debug, ss.str(), cv::Point(20, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
	}
}

void callback(char key, cv::Mat* image = NULL) {
	switch(key)
	{
		case 'w': fidDetector.lineVotes++; break;
		case 's': fidDetector.lineVotes--; break;
		case 'a': fidDetector.circleVotes-=10; break;
		case 'd': fidDetector.circleVotes+=10; break;
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
		case 'v': fidDetector.highThreshold+=10; fidDetector.circleThreshold+=10; break;
		case 'c': fidDetector.highThreshold-=10; fidDetector.circleThreshold-=10; break;
		case 'm': fidDetector.blur+=2; break;
		case 'n': fidDetector.blur-=2; break;
		case '.': fidDetector.sigma+=.1; break;
		case ',': fidDetector.sigma-=.1; break;
		case 'b': showValues=!showValues; break;
		case '/': showContours=!showContours; break;
		case 'p': showDebug=!showDebug; break;
		case 'y': imwrite("screenshot.png", *image); break;
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
			process(gray, debug);

			cv::imshow("Image", debug);
			key = cv::waitKey();
			callback(key, &debug);
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

			process(gray, frame);

			cv::imshow("Frame", frame);
			key = cv::waitKey(10);
			callback(key, &frame);
		}
	}

	return 0;
}
