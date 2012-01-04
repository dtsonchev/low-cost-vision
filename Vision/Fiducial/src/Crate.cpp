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
	this->fidPoints.assign(points.begin(), points.begin()+3);
}

Crate::Crate(std::string name, const std::vector<cv::Point2f>& points) {
	this->name = name;
	this->fidPoints.assign(points.begin(), points.begin()+3);
}

Crate::Crate(const Crate& crate) :
		bounds(crate.bounds), name(crate.name),
		fidPoints(crate.fidPoints) {
}

Crate::~Crate() {
}

cv::RotatedRect Crate::rect() {
	if(bounds.size.area() != 0.0f)
		return bounds;

	// Determine the distance between the fiducial points
	float distance1 = sqrt(pow(fidPoints[0].x - fidPoints[1].x, 2) + pow(fidPoints[0].y - fidPoints[1].y, 2));
	float distance2 = sqrt(pow(fidPoints[2].x - fidPoints[1].x, 2) + pow(fidPoints[2].y - fidPoints[1].y, 2));

	// Distance and angle between the diagonal points
	float length = sqrt(distance1*distance1 + distance2*distance2);
	float alpha = atan2(fidPoints[0].y - fidPoints[2].y, fidPoints[2].x - fidPoints[0].x);

	// Determine the center
	bounds.center = cv::Point2f(fidPoints[0].x + (length / 2.0) * cos(-alpha),
			fidPoints[0].y + (length / 2.0) * sin(-alpha));
	bounds.size = cv::Size(distance1, distance2);
	bounds.angle = alpha - acos(distance1/length);

	return bounds;
}

void Crate::draw(cv::Mat& image) {
	// Draw the fiducial points
	cv::circle(image, fidPoints[0], 1, cv::Scalar(255, 0, 0), 2);
	cv::circle(image, fidPoints[1], 1, cv::Scalar(0, 255, 0), 2);
	cv::circle(image, fidPoints[2], 1, cv::Scalar(0, 0, 255), 2);

	// Order the points and intialize the bounds
	cv::RotatedRect rect = this->rect();

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

	if(!name.empty()) {
		cv::putText(image, name, cv::Point(rect.center.x, rect.center.y-20), CV_FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,255), 2);
	}
}
