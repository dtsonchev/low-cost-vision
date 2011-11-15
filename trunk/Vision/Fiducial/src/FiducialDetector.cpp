/*
 * FiducialDetector.cpp
 *
 *  Created on: Nov 2, 2011
 *      Author: Jules Blok
 */

#include "FiducialDetector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <sstream>

using namespace cv;
using namespace std;

FiducialDetector::FiducialDetector(int minRad, int maxRad, int distance,
		int circleVotes, int minDist, int maxDist,
		int lineVotes, int lowThreshold, int highThreshold) {
	this->verbose = false;
	this->minRad = minRad;
	this->maxRad = maxRad;
	this->distance = distance;
	this->circleVotes = circleVotes;
	this->minDist = minDist;
	this->maxDist = maxDist;
	this->lineVotes = lineVotes;
	this->lowThreshold = lowThreshold;
	this->highThreshold = highThreshold;
}

FiducialDetector::~FiducialDetector() {
}

void FiducialDetector::polarLine(cv::Mat& image, float rho, float theta, cv::Scalar color, int thickness) {
	if (theta < M_PI / 4. || theta > 3. * M_PI / 4.) { // ~vertical line
		// point of intersection of the line with first row
		Point pt1(rho / cos(theta), 0);
		// point of intersection of the line with last row
		Point pt2((rho - image.rows * sin(theta)) / cos(theta),
				image.rows);
		// draw the line
		line(image, pt1, pt2, color, thickness);
	} else { // ~horizontal line
		// point of intersection of the line with first column
		Point pt1(0, rho / sin(theta));
		// point of intersection of the line with last column
		Point pt2(image.cols,
				(rho - image.cols * cos(theta)) / sin(theta));
		// draw the line
		line(image, pt1, pt2, color, thickness);
	}
}

void FiducialDetector::detect(cv::Mat& image, std::vector<cv::Point2f>& points, cv::Mat* debugImage) {
	// Detect circles
	vector<Vec3f> circles;
	HoughCircles(image, circles, CV_HOUGH_GRADIENT,
			1, // accumulator resolution divisor
			distance, // minimum distance between circles
			highThreshold, // Canny high threshold
			circleVotes, // minimum number of votes
			minRad, maxRad); // min and max radius

	// Accurately detect the center for every circle with sub-pixel precision
	for (vector<Vec3f>::const_iterator it = circles.begin();
			it != circles.end(); it++) {
		// Set ROI to the circle
		Point center((*it)[0], (*it)[1]);
		float rad = (*it)[2];
		Rect bounds(
				MAX(center.x - rad, 0), MAX(center.y - rad, 0),
				center.x+rad<image.cols?rad*2:image.cols-center.x,
				center.y+rad<image.rows?rad*2:image.rows-center.y);
		Mat roi = image(bounds);

		// Generate inverted circle mask
		Mat roiMask = Mat::zeros(roi.rows,roi.cols,CV_8U);
		Point roiCenter(roi.cols/2,roi.rows/2);
		circle(roiMask, roiCenter, rad-1, Scalar(255),-1);

		// If lines were found
		Point2f roiPoint;
		bool ret = false;
		if(debugImage!=NULL) {
			Mat roiDebug = (*debugImage)(bounds);
			ret = detectCrosshair(roi, roiPoint, roiMask, &roiDebug);
		}
		else {
			ret = detectCrosshair(roi, roiPoint, roiMask);
		}

		if(ret) {
			Point2f point(bounds.x + roiPoint.x, bounds.y + roiPoint.y);
			if(bounds.contains(point)) points.push_back(point);
			else if(verbose) cout << "Center: " << center << " outside ROI!" << endl;
		}

		// Draw the detected circles
		if(debugImage!=NULL) circle(*debugImage, Point((*it)[0], (*it)[1]), (*it)[2], Scalar(0, 255, 0), 2);
	}
}

bool FiducialDetector::detectCrosshair(cv::Mat& image, cv::Point2f& center, const cv::Mat& mask, cv::Mat* debugImage) {
	Mat contours;
	Canny(image, contours, lowThreshold, highThreshold);

	if(!mask.empty()) {
		Mat invMask;
		threshold(mask, invMask, 128, 255, CV_THRESH_BINARY_INV);
		contours.setTo(Scalar(0), invMask);
	}

	// Hough tranform for line detection
	vector<Vec2f> lines;
	HoughLines(contours, lines, 1,
		M_PI / 180.0, // step size
		lineVotes); // minimum number of votes

	if(lines.empty()) return false;

	// Segment perpendicular lines for center detection
	vector<Vec2f> lines1;
	vector<Vec2f> lines2;
	float referenceAngle = lines[0][1];

	// Segment the lines
	for(vector<Vec2f>::iterator it = lines.begin();
			it != lines.end();it++) {
		float dist = abs(referenceAngle - (*it)[1]);
		if(dist < M_PI/4.0) lines1.push_back(*it);
		else lines2.push_back(*it);

		// Draw a blue line
		if(debugImage!=NULL) polarLine(*debugImage, (*it)[0], (*it)[1], Scalar(255, 0, 0), 1);
	}

	// Find two lines through the cross center
	Vec2f center1;
	Vec2f center2;
	bool found1 = false;
	bool found2 = false;

	// If the are more than two lines we can detect the line through the center
	if(lines1.size() >= 2)
	{
		// Two lines representing the sides of the cross
		Vec2f line1;
		Vec2f line2;
		bool line1Found = false;

		// Calculate average angle
		float meanTheta = 0;
		for(vector<Vec2f>::iterator it=lines1.begin();it!=lines1.end(); it++) meanTheta += (*it)[1];
		meanTheta = meanTheta / lines1.size();

		// Select the first line
		float lastTheta = M_PI;
		for(vector<Vec2f>::iterator it=lines1.begin();it!=lines1.end(); it++)
		{
			// Check if this line has a better orientation and distance than the last
			float thetaDist = abs(meanTheta - (*it)[1]);
			if(thetaDist < lastTheta)
			{
				// Set the first line
				line1 = *it;
				lastTheta = thetaDist;
				line1Found = true;
			}
		}

		if(line1Found) {
			// Make sure the second line is on the opposite side
			lastTheta = M_PI;
			for(vector<Vec2f>::iterator it=lines1.begin(); it!=lines1.end(); it++)
			{
				// Check if this line has a better orientation than the last
				float thetaDist = abs(line1[1] - (*it)[1]);
				if(thetaDist < lastTheta)
				{
					// Check if the line is on the opposite side
					float rhoDist = abs(line1[0] - (*it)[0]);
					if(minDist < rhoDist && rhoDist <= maxDist)
					{
						// Set the second line to the opposite line
						line2 = *it;
						lastTheta = thetaDist;
						found1 = true;
					}
				}
			}

			if(found1) {
				float rho = (line1[0] + line2[0]) / 2.0;
				float theta = (line1[1] + line2[1]) / 2.0;
				center1 = Vec2f(rho, theta);

				// Draw a red line
				if(debugImage!=NULL) polarLine(*debugImage, rho, theta, Scalar(0,0,255), 1);
			}
			else if(verbose) cout << "Second primary line not found!" << endl;
		}
		else if(verbose) cout << "First primary line not found!" << endl;
	}
	else if(verbose) cout << "Not enough primary lines!" << endl;

	// If the are more than two lines we can detect the line through the center
	if(lines2.size() >= 2)
	{
		// Two lines representing the sides of the cross
		Vec2f line1;
		Vec2f line2;
		bool line1Found = false;

		// Calculate average angle
		float meanTheta = 0;
		for(vector<Vec2f>::iterator it=lines2.begin();it!=lines2.end(); it++) meanTheta += (*it)[1];
		meanTheta = meanTheta / lines2.size();

		// Select the first line
		float lastTheta = M_PI;
		for(vector<Vec2f>::iterator it=lines2.begin(); it!=lines2.end(); it++)
		{
			// Check if this line has a better orientation than the last
			float thetaDist = abs(meanTheta - (*it)[1]);
			if(thetaDist < lastTheta)
			{
				// Set the first line
				line1 = *it;
				lastTheta = thetaDist;
				line1Found = true;
			}
		}

		if(line1Found) {
			// Make sure the second line is on the opposite side
			lastTheta = M_PI;
			for(vector<Vec2f>::iterator it=lines2.begin(); it!=lines2.end(); it++)
			{
				// Check if this line has a better orientation than the last
				float thetaDist = abs(line1[1] - (*it)[1]);
				if(thetaDist < lastTheta)
				{
					// Check if the line is on the opposite side
					float rhoDist = abs(line1[0] - (*it)[0]);
					if(minDist <= rhoDist && rhoDist <= maxDist)
					{
						// Set the second line to the opposite line
						line2 = *it;
						lastTheta = thetaDist;
						found2 = true;
					}
				}
			}

			if(found2) {
				float rho = (line1[0] + line2[0]) / 2.0;
				float theta = (line1[1] + line2[1]) / 2.0;
				center2 = Vec2f(rho, theta);

				// Draw a red line
				if(debugImage!=NULL) polarLine(*debugImage, rho, theta, Scalar(0, 0, 255), 1);
			}
			else if(verbose) cout << "Second secondary line not found!" << endl;
		}
		else if(verbose) cout << "First secondary line not found!" << endl;
	}
	else if(verbose) cout << "Not enough secondary lines!" << endl;

	// If both have lines on opposing sides we can intersect them and find the center
	if(found1 && found2)
	{
		float rho1 = center1[0];
		float rho2 = center2[0];
		float omega1 = center1[1];
		float omega2 = center2[1];

		if(rho1 != 0 && rho2 != 0)
		{
			float theta = atan((cos(omega1) - (rho1/rho2)*cos(omega2))/-(sin(omega1)-(rho1/rho2)*sin(omega2)));
			float r = rho1 / cos(theta - omega1);
			center = Point2f (r*cos(theta), r*sin(theta));
		}
		else if(rho1 != 0)
		{
			// Line 2 is through the origin
			omega2 -= M_PI/2.0;
			if (omega1 < M_PI / 4. || omega1 > 3. * M_PI / 4.) {
				float y = rho1*(sin(omega2)/cos(omega1-omega2));
				float x = (rho1-y*sin(omega1))/cos(omega1);
				center = Point2f (x, y);
			} else { // ~horizontal line
				float x = rho1*(cos(omega2)/cos(omega1-omega2));
				float y = (rho1-x*cos(omega1))/sin(omega1);
				center = Point2f (x, y);
			}
		}
		else if(rho2 != 0)
		{
			// Line 1 is through the origin
			omega1 -= M_PI/2.0;
			if (omega2 < M_PI / 4. || omega2 > 3. * M_PI / 4.) {
				float y = rho2*(sin(omega1)/cos(omega2-omega1));
				float x = (rho2-y*sin(omega2))/cos(omega2);
				center = Point2f (x, y);
			} else { // ~horizontal line
				float x = rho2*(cos(omega1)/cos(omega2-omega1));
				float y = (rho2-x*cos(omega2))/sin(omega2);
				center = Point2f (x, y);
			}
		}
		else
		{
			// Both lines are through the origin
			center = Point2f(0,0);
		}

		if(debugImage!=NULL) circle(*debugImage, center, 1, Scalar(0, 0, 255), 2);

		return true;
	}
	return false;
}
