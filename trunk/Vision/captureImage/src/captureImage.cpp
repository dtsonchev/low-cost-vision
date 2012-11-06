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

#include <unicap_cv_bridge.hpp>
#include <CameraCalibration/RectifyImage.h>
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
	if(argc < 3) {
		printf("usage for starting capture: %s device_number format_number [calibration_xml]\ndevices:\n", argv[0]);
		print_devices_inventory();

		return 0;
	}

	int device_number = atoi(argv[1]);
	int format_number = atoi(argv[2]);

	unicap_cv_camera cam(device_number, format_number);
	Size camSize(cam.get_img_width(), cam.get_img_height());

	Mat undistorted;
	RectifyImage rectifier;
	if(argc > 3) rectifier.initRectify(argv[3], camSize);

	Mat frame(camSize, cam.get_img_format());
	Mat hist(600, 900, CV_8UC3);

	cam.set_auto_white_balance(true);

	int counter = 0;
	double exposure = 0.015;
	double blue, red;
	char key = 0;
	bool tripmode = false;
	bool undistort = argc > 3;

	while(key != 'q') {
		cam.get_frame(&frame);

		if(undistort) {
			rectifier.rectify(frame, undistorted);
			imshow("undistorted", undistorted);
		}
		else {
			imshow("distorted", frame);
		}

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
		}else if(key == 'c'){
			if(argc > 3) undistort = !undistort;
		}else if(key == 'b'){
			ss.str("");
			ss << "Image" << counter++ << ".jpg";
			if(undistort) imwrite(ss.str().c_str(), undistorted);
			else imwrite(ss.str().c_str(), frame);
			cout << "Image taken: " << ss.str().c_str() << endl;
			key = 0;	
		}
		cout.flush();
	}

	return 0;
}
