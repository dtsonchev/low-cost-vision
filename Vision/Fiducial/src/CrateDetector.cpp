/*
 * CrateDetector.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: Jules Blok
 */

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

void CrateDetector::detect(std::vector<Crate>& crates, const std::vector<cv::Point2f>& points, const cv::Mat& image) {
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
