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

/*! \brief Segements points into crates.
 *
 *  This class allows you to segement the fiducial
 *  points into instances of the Crate class.
 */
class CrateDetector {
public:
	//! First canny threshold
	int lowThreshold;
	//! Second canny threshold
	int highThreshold;

	/*! \brief The CrateDetector constructor
	 *
	 *  Constructs the crate detector with default thresholds.
	 *  Supply more arguments to change the thresholds or change
	 *  the public fields after construction.
	 */
	CrateDetector(int lowThreshold = 50, int highThreshold = 150);
	virtual ~CrateDetector();


	/*! \brief Segment the points into crates
	 *
	 *  Detects all crates in the image and segments points
	 *  belonging to each crate into Crate instances. These
	 *  are added to the crates vector.
	 *
	 *	\param crates Output vector for the crates
	 *  \param points Detected fiducial points
	 *  \param image Image with the crates
	 */
	void detect(const cv::Mat& image, std::vector<Crate>& crates, const std::vector<cv::Point2f>& points);
};

#endif /* CRATEDETECTOR_H_ */
