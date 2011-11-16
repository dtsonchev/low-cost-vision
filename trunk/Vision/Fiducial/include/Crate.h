/*
 * Crate.h
 *
 *  Created on: Nov 11, 2011
 *      Author: armada
 */

#ifndef CRATE_H_
#define CRATE_H_

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>

class Crate {
private:
	cv::RotatedRect bounds;
public:
	std::vector<cv::Point2f> points;

	Crate();
	Crate(const std::vector<cv::Point2f>& points);
	Crate(const Crate& crate);
	virtual ~Crate();

	cv::RotatedRect rect();
	void draw(cv::Mat& image);
};

#endif /* CRATE_H_ */
