//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Fiducial
// File:           FiducialDetector.h
// Description:    Detects fiduciary markers
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

#ifndef FIDUCIALDETECTOR_H_
#define FIDUCIALDETECTOR_H_

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <ros/ros.h>
#include "fiducial/fiducial_feedback.h"

/*! \brief Detects fiducial markers.
 *
 *  This class allows you to locate the cross.png
 *  and the crosshair.png fiducials in an image.
 */
class FiducialDetector {
private:
	//! Draw a polar coordinate line
	void polarLine(cv::Mat& image, float rho, float theta, cv::Scalar color,
			int thickness);
	//! Determine the center line using the mediod angle
	bool detectMedoidCenterLine(cv::Vec2f& centerLine, std::vector<cv::Vec2f> lines, cv::Mat* debugImage=NULL);
	//! Determine the center line using the mean angle
	bool detectMeanCenterLine(cv::Vec2f& centerLine, std::vector<cv::Vec2f> lines, cv::Mat* debugImage=NULL);
public:
	//! Center line detection methods
	enum {
		MEAN=0,
		MEDOID_THETA=1,
		MEDOID_RHO=2
	};

	//! Turn on console debug messages
	bool verbose;
	//! Minimum circle radius
	int minRad;
	//! Maximum circle radius
	int maxRad;
	//! Minimum distance between circles
	int distance;
	//! Vote threshold for circles
	int circleVotes;
	//! Minimum vote threshold for lines, thinner lines get less votes.
	int lineVotes;
	//! Maximum amount of lines that need to be found.
	unsigned int maxLines;
	//! Minimum distance between lines to use for the center line.
	float minDist;
	//! Maximum distance between lines to use for the center line.
	float maxDist;
	//! First canny threshold
	int lowThreshold;
	//! Second canny threshold
	int highThreshold;
	//! Center line detection method
	int centerMethod;

	/*! \brief The FiducialDetector constructor
	 *
	 *  Constructs the fiducial detector with default thresholds.
	 *  Supply more arguments to change the thresholds or change
	 *  the public fields after construction.
	 */
	FiducialDetector(int minRad = 20, int maxRad = 40, int distance = 70,
			int circleVotes = 100, float minDist = 1.5f, float maxDist = 5.0f,
			int lineVotes = 10, unsigned int maxLines = 10, int lowThreshold = 125,
			int highThreshold = 300, int centerMethod=MEAN);

	//! The FiducialDetector deconstructor
	virtual ~FiducialDetector();

	/*! \brief Detects all fiducials in an image
	 *
	 *  Detects all fiducials in the image and automatically
	 *  calls detectCrosshair for each fiducial adding the
	 *  center points to the points vector.
	 *
	 *  \param image Image with the fiducials
	 *  \param points Output vector that will contain the
	 *  center points
	 *  \param debugImage Output image where debug information
	 *  will be drawn on, set to NULL for no debug information
	 */
	void detect(cv::Mat& image, std::vector<cv::Point2f>& points,
			fiducial::fiducial_feedback* feedback = NULL,
			cv::Mat* debugImage = NULL);

	/*! \brief Detects the center point
	 *
	 *  Automatically called by detect(). After the fiducial has
	 *  been segmented this function will determine the center
	 *  point of the crosshair.
	 *
	 *  \sa detect()
	 *
	 *  \param image Image with the crosshair
	 *  \param center Output point that will be set to the
	 *  center point
	 *  \param mask Operation mask of the same size as image
	 *  \param debugImage Output image where debug information
	 *  will be drawn on, set to NULL for no debug information
	 *  \return <i>true</i> if center point was detected\n
	 *  <i>false</i> if detection failed
	 */
	bool detectCrosshair(cv::Mat& image, cv::Point2f& center,
			const cv::Mat& mask = cv::Mat(),
			fiducial::fiducial_feedback* feedback = NULL,
			cv::Mat* debugImage = NULL);
};

#endif /* FIDUCIALDETECTOR_H_ */
