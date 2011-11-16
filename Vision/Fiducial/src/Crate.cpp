/*
 * Crate.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: armada
 */

#include "Crate.h"
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

Crate::Crate() {
}

Crate::Crate(const std::vector<cv::Point2f>& points) {
	if(points.size() == 3) this->points.assign(points.begin(), points.begin()+3);
}

Crate::Crate(const Crate& crate) : bounds(crate.bounds), points(crate.points) {
}

Crate::~Crate() {
}

RotatedRect Crate::rect() {
	if(bounds.size.area() != 0.0f || points.size() != 3) return bounds;

	Point2f start;
	Point2f end;
	Point2f extra;
	float distance = 0;
	vector<Point2f>::iterator it;
	vector<Point2f>::iterator subIt;
	for(it = points.begin(); it!=points.end(); it++) {
		for(subIt = points.begin(); subIt!=points.end(); subIt++) {
			float dist = sqrt(pow(it->x - subIt->x, 2) + pow(it->y - subIt->y, 2));
			if(dist > distance)
			{
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
	bounds.center = Point2f(start.x + (distance / 2.0) * cos(-angle),
			start.y + (distance / 2.0) * sin(-angle));
	float orientation = atan2(bounds.center.y - extra.y, extra.x - bounds.center.x);
	bounds.angle = orientation - 3.0*M_PI/4.0;
	if(bounds.angle < -M_PI) bounds.angle += M_PI*2.0;

	float extraDistance = sqrt(pow(bounds.center.x - extra.x, 2) + pow(bounds.center.y - extra.y, 2));
	vector<Point2f> boundingPoints(points);
	Point2f boundPoint = Point2f(bounds.center.x + extraDistance*cos(-orientation-M_PI), bounds.center.y + extraDistance*sin(-orientation-M_PI));
	boundingPoints.push_back(boundPoint);
	bounds.size = boundingRect(boundingPoints).size();

	return bounds;
}

void Crate::draw(cv::Mat& image) {
	// Draw the angle points with a line
	for(vector<Point2f>::iterator it=points.begin();it!=points.end();++it) circle(image, *it, 1, Scalar(0, 0, 255), 2);

	RotatedRect rect = this->rect();

	// Draw arrow
	Point pt1 = rect.center;
	Point pt2(pt1.x - 50 * cos(-bounds.angle+M_PI/2.0),
			pt1.y - 50 * sin(-bounds.angle+M_PI/2.0));
	line(image, pt1, pt2, Scalar(0, 0, 0), 2);
	line(image, pt2,
			Point(pt2.x + 10 * cos(-bounds.angle+3*M_PI/4.0),
					pt2.y + 10 * sin(-bounds.angle+3*M_PI/4.0)),
			Scalar(0, 0, 0), 2);
	line(image, pt2,
			Point(pt2.x + 10 * cos(-bounds.angle+M_PI/4.0),
					pt2.y + 10 * sin(-bounds.angle+M_PI/4.0)),
			Scalar(0, 0, 0), 2);
	stringstream ss;
	ss << saturate_cast<int>(bounds.angle / (M_PI/180.0));
	putText(image, ss.str(), pt1-Point(15,0), CV_FONT_HERSHEY_SIMPLEX, .5, Scalar(255,0,0), 2);

	rectangle(image, rect.boundingRect(), Scalar(0,255,0), 2);
}
