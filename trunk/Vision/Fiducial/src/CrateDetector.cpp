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
#include <set>

CrateDetector::CrateDetector(int lowThreshold, int highThreshold) {
	this->lowThreshold = lowThreshold;
	this->highThreshold = highThreshold;
}

CrateDetector::~CrateDetector() {
}

void CrateDetector::order(std::vector<cv::Point2f>& points) {
	// Find the two diagonal opposite points
	float distance = 0;
	cv::Point2f start;
	cv::Point2f end;
	cv::Point2f extra;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			float dist = sqrt(pow(points[i].x - points[j].x, 2) + pow(points[i].y - points[j].y, 2));
			if(dist > distance) {
				distance = dist;
				if(points[i].x < points[j].x) {
					start = points[i];
					end = points[j];
				}
				else {
					start = points[j];
					end = points[i];
				}
			}
		}
	}

	for(int i=0; i<3; i++) {
		if(points[i] != start && points[i] != end) {
			extra = points[i];
		}
	}

	// Now the orientation of the crate can be detect and the first two points can be ordered
	float angle = (extra.x - start.x) * (end.y - start.y) - (extra.y - start.y) * (end.x - start.x);
	if(angle > 0) {
		if(start.x > end.x) {
			points[0] = end;
			points[1] = extra;
			points[2] = start;
		}
		else {
			points[0] = start;
			points[1] = extra;
			points[2] = end;
		}
	}
	else {
		if(start.x > end.x) {
			points[0] = start;
			points[1] = extra;
			points[2] = end;
		}
		else {
			points[0] = end;
			points[1] = extra;
			points[2] = start;
		}
	}
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
			order(fiducials);
			Crate crate(fiducials);
			crates.push_back(crate);
			usedPoints.insert(usedPoints.end(), fiducials.begin(), fiducials.end());
		}
	}

	for(std::vector<cv::Point2f>::const_iterator it = points.begin();
		it != points.end(); ++it) {
		std::vector<cv::Point2f>::iterator used_it = usedPoints.begin();
		for(; used_it!=usedPoints.end(); ++used_it) if(*it == *used_it) break;
		if(used_it == usedPoints.end()) leftovers->push_back(*it);
	}
}
