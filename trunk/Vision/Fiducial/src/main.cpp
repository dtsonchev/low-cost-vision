#include "FiducialDetector.h"
#include "CrateDetector.h"
#include "Crate.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

FiducialDetector fidDetector;
CrateDetector crateDetector;

void process(Mat& image, Mat& debug) {
	vector<Point2f> points;
	RotatedRect rect;
	fidDetector.detect(image, points, &debug);

	/*Mat canny;
	Canny(image, canny, crateDetector.lowThreshold, crateDetector.highThreshold);
	imshow("Canny", canny);

	vector<vector<Point> > contoursCanny;
	findContours(canny, contoursCanny, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	Mat cannyContours = Mat::zeros(image.rows, image.cols, CV_8UC3);

	for(unsigned int i=0; i<contoursCanny.size(); i++)
		drawContours(cannyContours, contoursCanny, i, Scalar(rand()%255,rand()%255,rand()%255), 2);
	imshow("Contours", cannyContours);*/

	if(points.size() > 3) {
		vector<Crate> crates;
		crateDetector.detect(crates, points, image);

		for(vector<Crate>::iterator it=crates.begin(); it!=crates.end(); ++it) it->draw(debug);
	}
	else if(points.size() == 3) {
		Crate crate(points);
		crate.draw(debug);
	}

    rectangle(debug, Point(10, 5), Point(210, 90), Scalar(100, 100, 100, 50), CV_FILLED, 1, 0);
	stringstream ss;
	ss << "Votes: " << fidDetector.lineVotes << " | " << fidDetector.circleVotes;
    cv::putText(debug, ss.str(), cv::Point(20, 20), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
    ss.str("");
    ss << "Circle radius: " << fidDetector.minRad << "/" << fidDetector.maxRad;
    cv::putText(debug, ss.str(), cv::Point(20, 40), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
    ss.str("");
    ss << "Line distance:  " << fidDetector.minDist << "/" << fidDetector.maxDist;
    cv::putText(debug, ss.str(), cv::Point(20, 60), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
    ss.str("");
    ss << "Threshold:  " << fidDetector.lowThreshold << "/" << fidDetector.highThreshold;
    cv::putText(debug, ss.str(), cv::Point(20, 80), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
}

void callback(char key) {
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
		case '>': crateDetector.highThreshold+=10; break;
		case '<': crateDetector.highThreshold-=10; break;
	}
}

int main(int argc, char* argv[]) {
	if(argc < 3) {
		cout << "Usage:\t" << argv[0] << " image <path>" << endl;
		cout << "\t" << argv[0] << " cam <index>" << endl;
		return 1;
	}

	fidDetector.verbose = true;

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
