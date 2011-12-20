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
// License:        GNU GPL v3
//
// This file is part of Fiducial.
//
// Fiducial is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Fiducial is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Fiducial.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include "CrateDetector.h"
#include "Crate.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

CrateDetector::CrateDetector(int lowThreshold, int highThreshold) {
	this->lowThreshold = lowThreshold;
	this->highThreshold = highThreshold;
}

CrateDetector::~CrateDetector() {
}

void CrateDetector::detect(const cv::Mat& image, std::vector<Crate>& crates, const std::vector<cv::Point2f>& points) {
	cv::Mat canny;
	cv::Canny(image, canny, lowThreshold, highThreshold);

	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

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
			Crate crate(fiducials);
			crates.push_back(crate);
		}
	}
}
