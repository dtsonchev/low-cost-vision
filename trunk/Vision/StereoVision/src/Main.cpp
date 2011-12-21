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
// License:        GNU GPL v3
//
// This file is part of StereoVision.
//
// StereoVision is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// StereoVision is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with StereoVision.  If not, see <http://www.gnu.org/licenses/>.
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
