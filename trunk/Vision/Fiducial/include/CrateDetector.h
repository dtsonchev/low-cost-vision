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
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

#ifndef CRATEDETECTOR_H_
#define CRATEDETECTOR_H_

#include "Crate.h"
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>

/*! \brief Segements points into crates.
 *
 *  \deprecated Support dropped in favor of the more
 *  robust QRCodeDetector for crate localization.
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
	 *  \param image Image with the crates
	 *  \param points Detected fiducial points
	 *	\param crates Output vector for the crates
	 *	\param leftovers Optional pointer to output vector
	 *	                 for the unused points
	 */
	void detect(const cv::Mat& image, const std::vector<cv::Point2f>& points, std::vector<Crate>& crates, std::vector<cv::Point2f>* leftovers = NULL);
};

#endif /* CRATEDETECTOR_H_ */
