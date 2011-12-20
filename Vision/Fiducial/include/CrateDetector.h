//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Fiducial
// File:           CrateDetector.h
// Description:    Segments point cloud into crates
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

	//! The CrateDetector deconstructor
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
