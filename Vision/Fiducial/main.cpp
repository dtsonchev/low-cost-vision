#include "FiducialDetector.h"
#include "Crate.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

FiducialDetector detector;

void process(Mat& image, Mat& debug) {
	vector<Point2f> points;
	RotatedRect rect;
	detector.detect(image, points, &debug);

	if(points.size() == 3) {
		Crate crate(points);
		crate.draw(debug);
	}


    rectangle(debug, Point(10, 5), Point(210, 90), Scalar(100, 100, 100, 50), CV_FILLED, 1, 0);
	stringstream ss;
	ss << "Votes: " << detector.lineVotes << " | " << detector.circleVotes;
    cv::putText(debug, ss.str(), cv::Point(20, 20), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
    ss.str("");
    ss << "Circle radius: " << detector.minRad << "/" << detector.maxRad;
    cv::putText(debug, ss.str(), cv::Point(20, 40), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
    ss.str("");
    ss << "Line distance:  " << detector.minDist << "/" << detector.maxDist;
    cv::putText(debug, ss.str(), cv::Point(20, 60), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
    ss.str("");
    ss << "Threshold:  " << detector.lowThreshold << "/" << detector.highThreshold;
    cv::putText(debug, ss.str(), cv::Point(20, 80), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
}

void callback(char key) {
	switch(key)
	{
		case 'w': detector.lineVotes++; break;
		case 's': detector.lineVotes--; break;
		case 'a': detector.circleVotes--; break;
		case 'd': detector.circleVotes++; break;
		case 'u': detector.minRad++; break;
		case 'j': detector.minRad--; break;
		case 'h': detector.maxRad--; break;
		case 'k': detector.maxRad++; break;
		case 'r': detector.minDist++; break;
		case 'f': detector.minDist--; break;
		case 't': detector.maxDist++; break;
		case 'g': detector.maxDist--; break;
		case 'x': detector.lowThreshold++; break;
		case 'z': detector.lowThreshold--; break;
		case 'v': detector.highThreshold++; break;
		case 'c': detector.highThreshold--; break;
	}
}

int main(int argc, char* argv[]) {
	if(argc < 3) {
		cout << "Usage:\t" << argv[0] << " image <path>" << endl;
		cout << "\t" << argv[0] << " cam <index>" << endl;
		return 1;
	}

	detector.minRad = 20;
	detector.maxRad = 150;
	detector.maxDist = 20;
	detector.verbose = true;

	if(!strcmp(argv[1], "image")) {
		Mat image = imread(argv[2], CV_LOAD_IMAGE_COLOR); // Read the file
		if (!image.data) // Check for invalid input
		{
			cout << "Could not open or find the image" << endl;
			return 1;
		}

		Mat gray;
		cvtColor(image, gray, CV_BGR2GRAY);

		// Retrieve camera frames
		char key = 0;
		while (key != 'q') {
			Mat debug = image.clone();
			process(gray, debug);

			imshow("Image", debug);
			key = waitKey();
			callback(key);
		}
	}
	else if(!strcmp(argv[1], "cam")) {
		// Initialize camera
		Mat frame;
		VideoCapture cam(atoi(argv[2]));

		if (!cam.isOpened()) {
			cout << "Failed to open camera" << endl;
			return -1;
		}

		// Retrieve camera frames
		char key = 0;
		while (key != 'q') {
			cam.read(frame);

			Mat gray;
			cvtColor(frame, gray, CV_BGR2GRAY);

			process(gray, frame);

			imshow("Frame", frame);
			key = waitKey(10);
			callback(key);
		}
	}

	return 0;
}
