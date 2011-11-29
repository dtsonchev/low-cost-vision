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
		this->points.assign(points.begin(), points.begin()+3);
}

Crate::Crate(const Crate& crate) : bounds(crate.bounds), points(crate.points) {
}

Crate::~Crate() {
}

cv::RotatedRect Crate::rect() {
	if(bounds.size.area() != 0.0f || points.size() != 3)
		return bounds;

	cv::Point2f start;
	cv::Point2f end;
	cv::Point2f extra;
	float distance = 0;
	std::vector<cv::Point2f>::iterator it;
	std::vector<cv::Point2f>::iterator subIt;
	for(it = points.begin(); it!=points.end(); it++) {
		for(subIt = points.begin(); subIt!=points.end(); subIt++) {
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
	for(it = points.begin(); it!=points.end(); it++) {
		if(*it != start && *it != end)
			extra = *it;
	}

	// Detect angles
	float angle = atan2(start.y - end.y, end.x - start.x);
	bounds.center = cv::Point2f(start.x + (distance / 2.0) * cos(-angle),
			start.y + (distance / 2.0) * sin(-angle));
	float orientation = atan2(bounds.center.y - extra.y, extra.x - bounds.center.x);
	bounds.angle = orientation - 3.0*M_PI/4.0;
	if(bounds.angle < -M_PI) bounds.angle += M_PI*2.0;

	float startDistance = sqrt(pow(start.x - extra.x, 2) + pow(start.y - extra.y, 2));
	float endDistance = sqrt(pow(end.x - extra.x, 2) + pow(end.y - extra.y, 2));
	bounds.size = cv::Size(startDistance, endDistance);

	return bounds;
}

void Crate::draw(cv::Mat& image) {
	// Draw the angle points with a line
	for(std::vector<cv::Point2f>::iterator it=points.begin();it!=points.end();++it)
		cv::circle(image, *it, 1, cv::Scalar(0, 0, 255), 2);

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
}
