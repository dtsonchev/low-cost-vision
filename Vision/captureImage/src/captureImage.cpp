//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        captureImage
// File:           captureImage.cpp
// Description:    An example in how to set the shutter time and the exposure, also is the image rotated
// Author:         Glenn Meerstra, Lukas Vermond and Kasper van Nieuwland
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of captureImage.
//
// captureImage is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// captureImage is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with captureImage.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <unicap_cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdio>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;
using namespace unicap_cv_bridge;

void draw_hist(Mat& dest, const Mat& src, int histSize)
{
	rectangle(dest, Point(0, 0), Point(dest.cols, dest.rows), Scalar(0, 0, 0, 0), CV_FILLED, 1, 0);
	// Separate the image in 3 places ( R, G and B )
	vector<Mat> rgb_planes;
	split(src, rgb_planes);

	// Set the ranges ( for R,G,B) )
	float range[] = { 0, 255 } ;
	const float* histRange = { range };

	Mat r_hist, g_hist, b_hist;

	// Compute the histograms:
	calcHist(&rgb_planes[0], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, true, false);
	calcHist(&rgb_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, true, false);
	calcHist(&rgb_planes[2], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, true, false);

	// Draw the histograms for R, G and B
	int hist_w = dest.cols, hist_h = dest.rows;
	int bin_w = cvRound((double)(hist_w)/histSize);

	float hist_max = 0;
	for(int i = 0; i < histSize; i++){
		int x = r_hist.at<float>(i);
		if(x > hist_max) { hist_max = x; }
		x = g_hist.at<float>(i);
		if(x > hist_max) { hist_max = x; }
		x = b_hist.at<float>(i);
		if(x > hist_max) { hist_max = x; }
	}

	r_hist *= (dest.rows / (double)hist_max);
	g_hist *= (dest.rows / (double)hist_max);
	b_hist *= (dest.rows / (double)hist_max);

	// Draw for each channel
	int i;
	for(i = 1; i < histSize; i++){
		line(
			dest,
			Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
			Scalar(255, 0, 0), 1, 8, 0);
		line(
			dest,
			Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
			Scalar(0, 255, 0), 1, 8, 0);
		line(
			dest,
			Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
			Scalar(0, 0, 255), 1, 8, 0);
	}

	line(
		dest,
		Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
		Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i - 1))),
		Scalar(255, 0, 0), 1, 8, 0);
	line(
		dest,
		Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
		Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i - 1))),
		Scalar(0, 255, 0), 1, 8, 0);
	line(
		dest,
		Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
		Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i - 1))),
		Scalar(0, 0, 255), 1, 8, 0);
}

int main(int argc, char** argv)
{
	if(argc == 1){
		printf("usage for starting capture: %s device_number format_number\ndevices:\n", argv[0]);
		print_devices_inventory();

		return 0;
	}else if (argc != 3){
		return 1;
	}

	int device_number = atoi(argv[1]);
	int format_number = atoi(argv[2]);

	unicap_cv_camera cam(device_number, format_number);

	Mat frame(cam.get_img_height(), cam.get_img_width(), cam.get_img_format());
	Mat hist(600, 900, CV_8UC3);

	cam.set_white_balance(1, 1);

	Mat cameraMatrix, distCoeffs;
	Mat undistorted, map1, map2, rotated;

	FileStorage fs("cameraMatrix.xml", FileStorage::READ);
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cv::Mat(), frame.size(), CV_32FC1, map1, map2);

	int counter = 0;
	double exposure = 0.03;
	double blue, red;
	char key = 0;
	bool tripmode = false;

	while(key != 'q'){

		cam.get_frame(&frame);
		remap(frame, undistorted, map1, map2, INTER_LINEAR);

		transpose(undistorted, rotated);
		flip(rotated, rotated, -1);
		
		draw_hist(hist, frame, 128);

		cam.get_white_balance(blue, red);

		rectangle(hist, Point(10, 5), Point(210, 67), Scalar(100, 100, 100, 50), CV_FILLED, 1, 0);

		stringstream ss;
		ss << "exposure: " << exposure;
		cv::putText(hist, ss.str(), cv::Point(20,20), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		ss.str("");
		ss << "blue: " << blue;
		cv::putText(hist, ss.str(), cv::Point(20, 40), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);
		ss.str("");
		ss << "red:  " << red;
		cv::putText(hist, ss.str(), cv::Point(20, 60), FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255, 0), 1, 1, false);

		imshow("histogram", hist);
		imshow("rotated", rotated);

		key = waitKey(10);
		if(key == 'w'){
			exposure *= 1.125;
			cam.set_exposure(exposure);
		}else if(key == 's'){
			exposure /= 1.125;
			cam.set_exposure(exposure);
		}else if(key == 'a'){
			cam.set_auto_white_balance(true);
		}else if(key == 'z'){
			cam.set_auto_white_balance(false);
		}else if(key == 't'){
			tripmode = !tripmode;
			cam.set_trip_mode(tripmode);
		}else if(key == 'b'){
			ss.str("");
			ss << "Image" << counter++ << ".jpg";
			imwrite(ss.str().c_str(), rotated);
			cout << "Image taken: " << ss.str().c_str() << endl;
			key = 0;	
		}
		cout.flush();
	}

	return 0;
}
