/*
 * Crate.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: Jules Blok
 */

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

Crate::Crate(const Crate& crate) : bounds(crate.bounds), fidPoints(crate.fidPoints) {
	fidOrdered = true;
}

Crate::~Crate() {
}

cv::RotatedRect Crate::rect() {
	if(bounds.size.area() != 0.0f || fidPoints.size() != 3)
		return bounds;

	// Order the points, this also sets some of the bounding box properties
	points();

	float distance1 = sqrt(pow(fidPoints[0].x - fidPoints[1].x, 2) + pow(fidPoints[0].y - fidPoints[1].y, 2));
	float distance2 = sqrt(pow(fidPoints[2].x - fidPoints[1].x, 2) + pow(fidPoints[2].y - fidPoints[1].y, 2));
	bounds.size = cv::Size(distance1, distance2);

	return bounds;
}

std::vector<cv::Point2f> Crate::points() {
	if(fidOrdered) return fidPoints;

	// Find the two diagonal opposite points
	float distance = 0;
	cv::Point2f start;
	cv::Point2f end;
	cv::Point2f extra;
	std::vector<cv::Point2f>::iterator it;
	std::vector<cv::Point2f>::iterator subIt;
	for(it = fidPoints.begin(); it!=fidPoints.end(); it++) {
		for(subIt = fidPoints.begin(); subIt!=fidPoints.end(); subIt++) {
			float dist = sqrt(pow(it->x - subIt->x, 2) + pow(it->y - subIt->y, 2));
			if(dist > distance) {
				distance = dist;
				if(it->x < subIt->x) {
					start = *it;
					end = *subIt;
				}
				else {
					start = *subIt;
					end = *it;
				}
			}
		}
	}

	for(it = fidPoints.begin(); it!=fidPoints.end(); it++) {
		if(*it != start && *it != end) {
			extra = *it;
		}
	}

	fidPoints[0] = start;
	fidPoints[1] = extra;
	fidPoints[2] = end;

	// If we have the two outer points the center can be detected regardless of the order
	float alpha = atan2(fidPoints[0].y - fidPoints[2].y, fidPoints[2].x - fidPoints[0].x);
	bounds.center = cv::Point2f(fidPoints[0].x + (distance / 2.0) * cos(-alpha),
			fidPoints[0].y + (distance / 2.0) * sin(-alpha));

	// Now the orientation of the crate can be detect and the first two points can be ordered
	float orientation = atan2(bounds.center.y - fidPoints[1].y, fidPoints[1].x - bounds.center.x);
	if(orientation > 0) {
		if(start.x > end.x) {
			fidPoints[2] = start;
			fidPoints[0] = end;
		}
	}
	else {
		if(start.x < end.x) {
			fidPoints[2] = start;
			fidPoints[0] = end;
		}
	}

	// Now that we know the orientation we can also set the bounding box angle
	bounds.angle = orientation - 3.0*M_PI/4.0;
	if(bounds.angle < -M_PI) bounds.angle += M_PI*2.0;

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
