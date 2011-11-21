#include "unicap_cv_bridge.hpp"
#include "FiducialDetector.h"
#include "CrateDetector.h"
#include "Crate.h"
#include "BarcodeDetector.h"
#include "RectifyImage.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

FiducialDetector fidDetector;
CrateDetector crateDetector;
DetectBarcode barDetector;

bool showValues = true;
bool showContours = false;
bool showDebug = true;

void process(cv::Mat& image, cv::Mat& debug) {
	std::vector<cv::Point2f> points;
	cv::RotatedRect rect;
	if(showDebug)
		fidDetector.detect(image, points, &debug);
	else
		fidDetector.detect(image, points);

	if(showContours) {
		cv::Mat canny;
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

	std::vector<Crate> crates;
	if(points.size() > 3) crateDetector.detect(image, crates, points);
	else if(points.size() == 3) crates.push_back(Crate(points));

	if(showDebug) {
		for(std::vector<Crate>::iterator it=crates.begin(); it!=crates.end(); ++it) {
			cv::RotatedRect rect = it->rect();
			rect.size = cv::Size(rect.size.width * 2, rect.size.height * 2);
			cv::Rect bounds = rect.boundingRect();
			if ((bounds.tl().x >= 0 && bounds.tl().y >= 0)
					&& (bounds.br().x < image.cols && bounds.br().y < image.rows)) {
				cv::Mat roi = image(bounds);

				cv::rectangle(debug, bounds, cv::Scalar(255, 0, 0), 2);

				std::string result;
				barDetector.detect(roi, result);
				it->draw(debug);
				if(!result.empty()) {
					cv::putText(debug, result, cv::Point(rect.center.x, rect.center.y-20), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,255), 2);
				}
			}
		}
	}

	if(showValues) {
		cv::rectangle(debug, cv::Point(10, 5), cv::Point(210, 90), cv::Scalar(100, 100, 100, 50), CV_FILLED, 1, 0);
		std::stringstream ss;
		ss << "Votes: " << fidDetector.lineVotes << " | " << fidDetector.circleVotes;
		cv::putText(debug, ss.str(), cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		ss.str("");
		ss << "Circle radius: " << fidDetector.minRad << "/" << fidDetector.maxRad;
		cv::putText(debug, ss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		ss.str("");
		ss << "Line distance:  " << fidDetector.minDist << "/" << fidDetector.maxDist;
		cv::putText(debug, ss.str(), cv::Point(20, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		ss.str("");
		ss << "Threshold:  " << fidDetector.lowThreshold << "/" << fidDetector.highThreshold;
		cv::putText(debug, ss.str(), cv::Point(20, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
	}
}

void callback(char key, cv::Mat* image = NULL) {
	switch(key)
	{
		case 'w': fidDetector.lineVotes++; break;
		case 's': fidDetector.lineVotes--; break;
		case 'a': fidDetector.circleVotes--; break;
		case 'd': fidDetector.circleVotes++; break;
		case 'u': fidDetector.minRad++; break;
		case 'j': fidDetector.minRad--; break;
		case 'h': fidDetector.maxRad--; break;
		case 'k': fidDetector.maxRad++; break;
		case 'r': fidDetector.minDist++; break;
		case 'f': fidDetector.minDist--; break;
		case 't': fidDetector.maxDist++; break;
		case 'g': fidDetector.maxDist--; break;
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
	}
}

void usage(char* cmd) {
	std::cout << "Usage:\t" << cmd << " image <path> [rectify path]" << std::endl;
	std::cout << "\t" << cmd << " cam <index> [rectify path]" << std::endl;
	std::cout << "\t" << cmd << " unicap <index> <format> [rectify path]" << std::endl;
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

		RectifyImage rectify;
		if(argc > 3) {
			rectify.initRectify(argv[3], cv::Size(image.cols, image.rows));
			rectify.rectify(image.clone(), image);
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
			callback(key, &image);
		}
	}
	else if(!strcmp(argv[1], "cam")) {
		if(argc < 3) {
			usage(argv[0]);
			return 1;
		}

		// Initialize camera
		cv::VideoCapture cam(atoi(argv[2]));
		cv::Size size(cam.get(CV_CAP_PROP_FRAME_WIDTH), cam.get(CV_CAP_PROP_FRAME_HEIGHT));
		cv::Mat frame;

		if (!cam.isOpened()) {
			std::cout << "Failed to open camera" << std::endl;
			return -1;
		}

		RectifyImage rectify;
		if(argc > 3) rectify.initRectify(argv[3], size);

		// Retrieve camera frames
		char key = 0;
		while (key != 'q') {
			cam.read(frame);

			if(argc > 4) rectify.rectify(frame.clone(), frame);

			cv::Mat gray;
			cv::cvtColor(frame, gray, CV_BGR2GRAY);

			process(gray, frame);

			cv::imshow("Frame", frame);
			key = cv::waitKey(10);
			callback(key, &frame);
		}
	}
	else if(!strcmp(argv[1], "unicap")) {
		if(argc < 4) {
			usage(argv[0]);
			return 1;
		}

		// Initialize camera
		int device = atoi(argv[2]);
		int format = atoi(argv[3]);
		unicap_cv_bridge::unicap_cv_camera cam(device,format);
		cv::Size size(cam.get_img_width(), cam.get_img_height());
		cv::Mat frame(size, cam.get_img_format());

		cam.set_exposure(0.01);
		cam.set_auto_white_balance(true);

		RectifyImage rectify;
		if(argc > 4) rectify.initRectify(argv[4], size);

		// Retrieve camera frames
		char key = 0;
		while (key != 'q') {
			cam.get_frame(&frame);

			if(argc > 4) rectify.rectify(frame.clone(), frame);

			cv::Mat gray;
			cv::cvtColor(frame, gray, CV_BGR2GRAY);

			process(gray, frame);

			imshow("Frame", frame);
			key = cv::waitKey(10);
			callback(key, &frame);
		}
	}
	else {
		usage(argv[0]);
		return 1;
	}

	return 0;
}
