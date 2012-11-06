//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        StereoVision
// File:           StereoVision.cpp
// Description:    class to obtain depth information from two 2D images
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
#include <cstdio>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <StereoVision/StereoVision.hpp>

using namespace cv;
using namespace std;
namespace stereoVision{

StereoVision::StereoVision(cv::Size sz, std::string xmlPath) {
	if (xmlPath != "") {
		this->imgL = cv::Mat::zeros(sz, CV_8UC3);
		this->imgR = cv::Mat::zeros(sz, CV_8UC3);
		Rect validRoi[2];

		FileStorage fs(xmlPath, CV_STORAGE_READ);
		if (fs.isOpened()) {
			fs["RMAP00"] >> rmap[0][0];
			fs["RMAP01"] >> rmap[0][1];
			fs["RMAP10"] >> rmap[1][0];
			fs["RMAP11"] >> rmap[1][1];

			fs["ROI1_X"] >> validRoi[0].x;
			fs["ROI2_X"] >> validRoi[1].x;
			fs["ROI1_Y"] >> validRoi[0].y;
			fs["ROI2_Y"] >> validRoi[1].y;
			fs["ROI1_W"] >> validRoi[0].width;
			fs["ROI2_W"] >> validRoi[1].width;
			fs["ROI1_H"] >> validRoi[0].height;
			fs["ROI2_H"] >> validRoi[1].height;
			double sf = 600. / MAX(sz.width, sz.height);
			roiL = Rect(cvRound(validRoi[0].x * sf), cvRound(validRoi[0].y * sf), cvRound(validRoi[0].width * sf), cvRound(validRoi[0].height * sf));
			roiR = Rect(cvRound(validRoi[1].x * sf), cvRound(validRoi[1].y * sf), cvRound(validRoi[1].width * sf), cvRound(validRoi[1].height * sf));
		} else {
			cerr << "Could not open xml file\n";
			exit(-1);
		}
		fs.release();
		resetVariables();
	} else {
		cerr << "Empty xml file path\n";
		exit(-2);

	}
}

void updateSGBM(int, void* stereovisiondepthobject) {
	StereoVision* selfPtr = (StereoVision*) stereovisiondepthobject;
	selfPtr->sgbm.SADWindowSize = ((selfPtr->sgbm.SADWindowSize <= 0 ? 1 : selfPtr->sgbm.SADWindowSize) | 1);

	int noOfChannels = selfPtr->imgL.channels();

	selfPtr->sgbm.P1 = 8 * noOfChannels * selfPtr->sgbm.SADWindowSize * selfPtr->sgbm.SADWindowSize;
	selfPtr->sgbm.P2 = 32 * noOfChannels * selfPtr->sgbm.SADWindowSize * selfPtr->sgbm.SADWindowSize;
	selfPtr->sgbm.numberOfDisparities = ((selfPtr->sgbm.numberOfDisparities <= 15 ? 16 : selfPtr->sgbm.numberOfDisparities) / 16) * 16;
	selfPtr->sgbm.speckleRange = (selfPtr->sgbm.speckleRange / 16) * 16;
}

void StereoVision::showTrackBars(std::string windowName) {
	createTrackbar("preFilterCap", windowName, &sgbm.preFilterCap, 255, updateSGBM, this);
	createTrackbar("SADWindowSize", windowName, &sgbm.SADWindowSize, 33, updateSGBM, this);
	createTrackbar("minDisparity", windowName, &sgbm.minDisparity, 30, updateSGBM, this);
	createTrackbar("numberOfDisparities", windowName, &sgbm.numberOfDisparities, 255, updateSGBM, this);
	createTrackbar("uniquenessRatio", windowName, &sgbm.uniquenessRatio, 50, updateSGBM, this);
	createTrackbar("speckleWindowSize", windowName, &sgbm.speckleWindowSize, 300, updateSGBM, this);
	createTrackbar("speckleRange", windowName, &sgbm.speckleRange, 2555, updateSGBM, this);
	createTrackbar("disp12MaxDiff", windowName, &sgbm.disp12MaxDiff, 50, updateSGBM, this);
}

void StereoVision::resetVariables() {
	sgbm.preFilterCap = 0;
	sgbm.SADWindowSize = 0;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = 0;
	sgbm.uniquenessRatio = 0;
	sgbm.speckleWindowSize = 0;
	sgbm.speckleRange = 0;
	sgbm.disp12MaxDiff = 0;
}

cv::Mat StereoVision::createDepthImage(cv::Mat& leftImage, cv::Mat& rightImage) {
	remap(leftImage, imgL, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
	remap(rightImage, imgR, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);

	Mat temp;
	sgbm.fullDP = 0;
	sgbm.disp12MaxDiff = 1;
	sgbm.numberOfDisparities = (sgbm.numberOfDisparities == 0 ? 16 : sgbm.numberOfDisparities);
	sgbm(imgL, imgR, temp);
	temp.convertTo(resultSGBM, CV_8U, 255 / (sgbm.numberOfDisparities * 16.));

	return resultSGBM;
}

}
