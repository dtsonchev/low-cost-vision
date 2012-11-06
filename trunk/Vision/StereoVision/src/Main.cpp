//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        StereoVision
// File:           Main.cpp
// Description:    creates and uses a matrix in order to rectify an image the matrix is created by loading multiple sets of images with checker board pattern. Then obtains depth information from two 2D images
// Author:         Franc Pape & Wouter Langerak
// Notes:          
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

#include <iostream>
#include <vector>
#include <cstdio>
#include <cstring>
#include <limits>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <UEyeOpenCV.hpp>
#include <StereoVision/StereoVisionCalibration.hpp>
#include <StereoVision/StereoVision.hpp>

#define SQUARES_WIDTH 9
#define SQUARES_HEIGHT 6

using namespace std;

int main(int argc, char** argv) {
	namespace sv = stereoVision;
	sv::StereoVisionCalibration rs;

	int savedImagesCounter = 0;
	char key = '0';
	UeyeOpencvCam uCamL(640, 480);
	UeyeOpencvCam uCamR(640, 480);
	cv::Mat imageL, imageR;
	vector<cv::Mat> trainingImages;
	cout << "Create new stereo.xml? (y/n)" << endl;
	string newXml;
	cin >> newXml;
	cvStartWindowThread();
	if(newXml == "y") {
	cvNamedWindow("LEFT", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("RIGHT", CV_WINDOW_AUTOSIZE);
		while (key != '\n') {
			imageL = uCamL.getFrame();
			imageR = uCamR.getFrame();
			cv::imshow("LEFT", imageL);
			cv::imshow("RIGHT", imageR);

			key = cv::waitKey(10);

			if (key == ' ') {
				(cout << "[" << ++savedImagesCounter << "]\tSaving images...").flush();
				cv::cvtColor(imageL, imageL, CV_RGB2GRAY);
				cv::cvtColor(imageR, imageR, CV_RGB2GRAY);
				trainingImages.push_back(imageL);
				trainingImages.push_back(imageR);
				cout << "done" << endl;
			}
		}

		cv::destroyAllWindows();

		if (!trainingImages.empty()) {
			cout << "training..." << endl;
			if (rs.StereoCalib(trainingImages, "stereo.xml", cv::Size(SQUARES_WIDTH, SQUARES_HEIGHT))) {
				cout << "Training successfully finished" << endl;
			} else {
				cout << "Training failed" << endl;
				return -1;
			}

		}
	}

	sv::StereoVision svd(cv::Size(640, 480), "stereo.xml");
	cvNamedWindow("LEFT", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("RIGHT", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("result", CV_WINDOW_AUTOSIZE);
	svd.showTrackBars("result");
	key = 0;
	while (key != '\n') {
		imageL = uCamL.getFrame();
		imageR = uCamR.getFrame();
		cv::imshow("LEFT", imageL);
		cv::imshow("RIGHT", imageR);
		key = cv::waitKey(10);
		cv::imshow("result", svd.createDepthImage(imageL, imageR));
	}

	return 0;
}
