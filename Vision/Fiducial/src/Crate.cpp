//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Fiducial
// File:           Crate.cpp
// Description:    Container class for a single productcrate
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

#include "Crate.h"
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

Crate::Crate() {
}

Crate::Crate(const std::vector<cv::Point2f>& points) {
	if(points.size() == 3)
		this->fidPoints.assign(points.begin(), points.begin()+3);
	fidOrdered = false;
}

Crate::Crate(const Crate& crate) : bounds(crate.bounds), fidPoints(crate.fidPoints), fidOrdered(crate.fidOrdered) {
}

Crate::~Crate() {
}

void Crate::order(std::vector<cv::Point2f>& points, cv::Point2f* center, float* orientation) {
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

	// If we have the two outer points the center can be detected regardless of the order
	float alpha = atan2(start.y - end.y, end.x - start.x);
	cv::Point2f centerPoint(start.x + (distance / 2.0) * cos(-alpha),
			start.y + (distance / 2.0) * sin(-alpha));

	// Now the orientation of the crate can be detect and the first two points can be ordered
	float angle = atan2(centerPoint.y - extra.y, extra.x - centerPoint.x);
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

	// Now that we know the angle we can also set the orientation
	if(orientation != NULL) {
		*orientation = angle - 3.0*M_PI/4.0;
		if(*orientation < -M_PI) *orientation += M_PI*2.0;
	}

	// We have also determined the center
	if(center != NULL) *center = centerPoint;
}

cv::RotatedRect Crate::rect() {
	if(bounds.size.area() != 0.0f || fidPoints.size() != 3)
		return bounds;

	// Order the fiducial points and initialize some of the bounding properties
	points();

	// Measure the distance between the fiducial points
	float distance1 = sqrt(pow(fidPoints[0].x - fidPoints[1].x, 2) + pow(fidPoints[0].y - fidPoints[1].y, 2));
	float distance2 = sqrt(pow(fidPoints[2].x - fidPoints[1].x, 2) + pow(fidPoints[2].y - fidPoints[1].y, 2));

	// Multiply by the total size of the crate divided by the real distance between the fiducial points
	bounds.size = cv::Size(distance1*(5.0/3.15), distance2*(5.0/3.15));

	return bounds;
}

std::vector<cv::Point2f> Crate::points() {
	if(fidOrdered) return fidPoints;

	// Order the fiducial points and initialize some of the bounding properties
	order(fidPoints, &bounds.center, &bounds.angle);

	// Finished ordering, ensures it will not be done twice
	fidOrdered = true;

	return fidPoints;
}

void Crate::draw(cv::Mat& image) {
	// Order the points and intialize the bounds
	cv::RotatedRect rect = this->rect();

	// Draw the fiducial points
	cv::circle(image, fidPoints[0], 1, cv::Scalar(255, 0, 0), 2);
	cv::circle(image, fidPoints[1], 1, cv::Scalar(0, 255, 0), 2);
	cv::circle(image, fidPoints[2], 1, cv::Scalar(0, 0, 255), 2);

	// Draw rect
	{
		cv::Point2f pt1(
				rect.center.x + (rect.size.width / 2.0) * cos(-rect.angle)
						- (rect.size.height / 2.0) * sin(-rect.angle),
				rect.center.y + (rect.size.height / 2.0) * cos(-rect.angle)
						+ (rect.size.width / 2.0) * sin(-rect.angle));
		cv::Point2f pt2(
				rect.center.x - (rect.size.width / 2.0) * cos(-rect.angle)
						- (rect.size.height / 2.0) * sin(-rect.angle),
				rect.center.y + (rect.size.height / 2.0) * cos(-rect.angle)
						- (rect.size.width / 2.0) * sin(-rect.angle));
		cv::Point2f pt3(
				rect.center.x - (rect.size.width / 2.0) * cos(-rect.angle)
						+ (rect.size.height / 2.0) * sin(-rect.angle),
				rect.center.y - (rect.size.height / 2.0) * cos(-rect.angle)
						- (rect.size.width / 2.0) * sin(-rect.angle));
		cv::Point2f pt4(
				rect.center.x + (rect.size.width / 2.0) * cos(-rect.angle)
						+ (rect.size.height / 2.0) * sin(-rect.angle),
				rect.center.y - (rect.size.height / 2.0) * cos(-rect.angle)
						+ (rect.size.width / 2.0) * sin(-rect.angle));

		cv::line(image, pt1, pt2, cv::Scalar(0,255,0), 2);
		cv::line(image, pt2, pt3, cv::Scalar(0,255,0), 2);
		cv::line(image, pt3, pt4, cv::Scalar(0,255,0), 2);
		cv::line(image, pt4, pt1, cv::Scalar(0,255,0), 2);
	}

	// Draw arrow
	{
		cv::Point pt1 = rect.center;
		cv::Point pt2(pt1.x - 50 * cos(-rect.angle+M_PI/2.0),
				pt1.y - 50 * sin(-rect.angle+M_PI/2.0));
		cv::line(image, pt1, pt2, cv::Scalar(0, 0, 0), 2);
		cv::line(image, pt2,
				cv::Point(pt2.x + 10 * cos(-rect.angle+3*M_PI/4.0),
						pt2.y + 10 * sin(-rect.angle+3*M_PI/4.0)),
						cv::Scalar(0, 0, 0), 2);
		cv::line(image, pt2,
				cv::Point(pt2.x + 10 * cos(-rect.angle+M_PI/4.0),
						pt2.y + 10 * sin(-rect.angle+M_PI/4.0)),
						cv::Scalar(0, 0, 0), 2);
		std::stringstream ss;
		ss << cv::saturate_cast<int>(rect.angle / (M_PI/180.0));
		cv::putText(image, ss.str(), pt1-cv::Point(15,0), CV_FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255,0,0), 2);
	}
}
