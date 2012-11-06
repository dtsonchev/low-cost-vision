//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DifferentsesSeparation
// File:           liblocator.cpp
// Description:    Library for marking differnces between images
// Author:         Glenn Meerstra & Zep Mouris
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
#include <Locator/liblocator.h>
#include <Locator/cameraException.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <limits>
#include <iostream>

Locator::Locator(int device){
	cam = cv::VideoCapture(device);
	setStdVars();
	if(!cam.isOpened()){
		throw cameraException("can not open the camera");
	}
}
Locator::~Locator(){
	cam.release();

	lastCapturedFrame.release();
	detectedDifference.release();
	lastStableBackground.release();
	matchingBackground.release();

}

void Locator::setBackground(cv::Mat &stableBackground){
	cam.read(lastCapturedFrame);
	stableBackground = lastCapturedFrame.clone();
	lastStableBackground = lastCapturedFrame.clone();
	matchingBackground =lastCapturedFrame.clone();
	cam.read(lastCapturedFrame);
	maxNorm = cv::norm(stableBackground, lastCapturedFrame, cv::NORM_INF) + normRange;
}

void Locator::WaitForStableViewAndTakeImage(cv::Mat &stableImage){
	cam.read(lastCapturedFrame);
	norm = cv::norm(lastStableBackground, lastCapturedFrame, cv::NORM_INF);
	while(norm < maxNorm){
		cam.read(lastCapturedFrame);
		norm = cv::norm(lastStableBackground, lastCapturedFrame, cv::NORM_INF);
		
	}
	while(norm > maxNorm){
		cv::Mat detectedImage = lastCapturedFrame.clone();
		cv::waitKey(100); //100 is an amount of milliseconds
		cam.read(lastCapturedFrame);	
		norm = cv::norm(detectedImage, lastCapturedFrame, cv::NORM_INF);	
	}
	lastStableBackground = lastCapturedFrame.clone();
	stableImage = lastCapturedFrame.clone();
}

/*
 * The function calculates the difference between two images and shows the difference
 * on the difference pointer. the calculations are done by looking at the RGB values of the images
 */
void Locator::showDifference(cv::Mat &difference){
	difference = lastStableBackground.clone();
	cv::MatConstIterator_<cv::Vec3b> it = matchingBackground.begin<cv::Vec3b>(), it_end = matchingBackground.end<cv::Vec3b>();
	cv::MatIterator_<cv::Vec3b> _it = lastStableBackground.begin<cv::Vec3b>();
	cv::MatIterator_<cv::Vec3b> dst_it = difference.begin<cv::Vec3b>();
	cv::Vec3b pixBackground = *it;
	cv::Vec3b pixShot = *_it;
	int R =0,G = 0,B=0;
	//for every pixel in the image
	for (; it != it_end; ++it, ++_it ,++dst_it){
		pixBackground = *it;
		pixShot = *_it;
		B = abs(pixBackground[0]-pixShot[0]);
		G = abs(pixBackground[1]-pixShot[1]);
		R = abs(pixBackground[2]-pixShot[2]);
		if(R < differents||G < differents ||B < differents ){
			//the pixel is not different against the background.
			*dst_it = cv::Vec3b(0, 0, 0);
		}else{
			//the pixel is different from the background
			*dst_it = cv::Vec3b(255, 255, 255);
		}
	}
	cv::dilate(difference, difference, cv::Mat(), cv::Point(-1, -1), filterIterations);
	cv::erode(difference, difference, cv::Mat(), cv::Point(-1, -1), filterIterations);
	detectedDifference = difference.clone();
	
}

/*
 * Finds and draws blobs by looking at the contours
 * The image pointer needs to be an binary image
 * Rectangles are drawn on the original with objects
 * This function calculates the center as well
 */
void Locator::findAndDrawBlobs(cv::Mat &blobs ){
	blobs = lastStableBackground.clone();
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
	cv::Mat gray;
	cv::cvtColor(detectedDifference, gray, CV_BGR2GRAY);

	
	/// Find contours
	cv::findContours( gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	/// Approximate contours to polygons + get bounding rects and circles
	cv::vector<cv::vector<cv::Point> > contours_poly( contours.size() );
	cv::vector<cv::Rect> boundRect( contours.size() );
	cv::vector<cv::Point2f>center( contours.size() );
	cv::vector<float>radius( contours.size() );

	for( uint i = 0; i < contours.size(); i++ ){
		cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
		cv::minEnclosingCircle( contours_poly[i], center[i], radius[i] );
	}
	
	objectLocations.clear();
	/// Draw polygonal contour + bonding rects
	for( uint i = 0; i< contours.size(); i++ ){
		if(contours[i].size() > (uint)minContourSize){
			objectLocations.push_back(boundRect[i]);
			rectangle( blobs, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0,0,255), 2, 8, 0 );
			rectangle( blobs, cv::Point(center[i].x - 3, center[i].y - 3), cv::Point(center[i].x + 3, center[i].y + 3), cv::Scalar(0,255,255), 2, 8, 0 );
		}
	};
	gray.release();
}

