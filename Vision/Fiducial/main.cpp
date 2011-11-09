#include "FiducialDetector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
	if(argc < 3) {
		cout << "Usage:\t" << argv[0] << " image <path>" << endl;
		cout << "\t" << argv[0] << " cam <index>" << endl;
		return 1;
	}

	FiducialDetector detector;
	detector.verbose = true;

	if(!strcmp(argv[1], "image")) {
		Mat image = imread(argv[2], CV_LOAD_IMAGE_COLOR); // Read the file
		if (!image.data) // Check for invalid input
		{
			cout << "Could not open or find the image" << endl;
			return 1;
		}
		//flip(image, image, -1);

		Mat gray;
		cvtColor(image, gray, CV_BGR2GRAY);

		vector<Point2f> points;
		RotatedRect rect;
		detector.detect(gray, points, &image);
		detector.getRotatedRect(points, rect, &image);

		imshow("Image", image);
		waitKey(0);
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

			vector<Point2f> points;
			RotatedRect rect;
			detector.detect(gray, points, &frame);
			detector.getRotatedRect(points, rect, &frame);

			imshow("Frame", frame);
			key = waitKey(10);
		}
	}

	return 0;
}
