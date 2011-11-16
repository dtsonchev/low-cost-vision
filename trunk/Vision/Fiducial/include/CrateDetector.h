/*
 * CrateDetector.h
 *
 *  Created on: Nov 11, 2011
 *      Author: Jules Blok
 */

#ifndef CRATEDETECTOR_H_
#define CRATEDETECTOR_H_

#include "Crate.h"
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>

class CrateDetector {
public:
	int lowThreshold;
	int highThreshold;

	CrateDetector(int lowThreshold = 300, int highThreshold = 800);
	virtual ~CrateDetector();

	void detect(std::vector<Crate>& crates, const std::vector<cv::Point2f>& points, const cv::Mat& image);
};

#endif /* CRATEDETECTOR_H_ */
