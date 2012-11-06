//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Fiducial
// File:           CrateDetector.cpp
// Description:    Segments point cloud into crates
// Author:         Jules Blok
// Notes:          None
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

#include "CrateDetector.h"
#include "Crate.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <set>

CrateDetector::CrateDetector(int lowThreshold, int highThreshold) {
	this->lowThreshold = lowThreshold;
	this->highThreshold = highThreshold;
}

CrateDetector::~CrateDetector() {
}

void CrateDetector::detect(const cv::Mat& image, const std::vector<cv::Point2f>& points, std::vector<Crate>& crates, std::vector<cv::Point2f>* leftovers) {
	cv::Mat canny;
	cv::Canny(image, canny, lowThreshold, highThreshold);

	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point2f> usedPoints;

	for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin();
			it != contours.end(); ++it) {
		std::vector<cv::Point2f> fiducials;
		for (std::vector<cv::Point2f>::const_iterator point_it = points.begin();
				point_it != points.end(); ++point_it) {
			if (cv::pointPolygonTest(*it, *point_it, false) == 1) {
				fiducials.push_back(*point_it);
			}
		}
		if (fiducials.size() == 3) {
			Crate::order(fiducials);
			Crate crate(fiducials);
			crates.push_back(crate);
			usedPoints.insert(usedPoints.end(), fiducials.begin(), fiducials.end());
		}
	}

	if(leftovers != NULL) {
		for(std::vector<cv::Point2f>::const_iterator it = points.begin();
			it != points.end(); ++it) {
			std::vector<cv::Point2f>::iterator used_it = usedPoints.begin();
			for(; used_it!=usedPoints.end(); ++used_it) if(*it == *used_it) break;
			if(used_it == usedPoints.end()) leftovers->push_back(*it);
		}
	}
}
