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
	//! Determine the center line
	bool detectCenterLine(cv::Vec2f& centerLine, std::vector<cv::Vec2f> lines, cv::Mat* debugImage=NULL);
public:
	//! Turn on console debug messages
	bool verbose;

	//! Gaussian blur size
	int blur;
	//! Gaussian blur sigma
	double sigma;
	//! Vote threshold for circles
	int circleVotes;
	//! Minimum distance between circles
	double distance;
	//! Minimum circle radius
	int minRad;
	//! Maximum circle radius
	int maxRad;
	//! High canny threshold for circle detection (low threshold is twice smaller)
	double circleThreshold;

	//! Starting vote threshold for lines, thinner lines get less votes.
	int lineVotes;
	//! Maximum amount of lines that can be found.
	unsigned int maxLines;
	//! Minimum distance between lines to use for the center line.
	float minDist;
	//! Maximum distance between lines to use for the center line.
	float maxDist;
	//! Low canny threshold for line detection
	double lowThreshold;
	//! High canny threshold for line detection
	double highThreshold;

	/*! \brief The FiducialDetector constructor
	 *
	 *  Constructs the fiducial detector with default properties.
	 *  The minimum and maximum radius can be changed in the constructor,
	 *  all other properties can be changed after construction.
	 */
	FiducialDetector(int minRad = 20, int maxRad = 40);

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
			const cv::Mat& mask = cv::Mat(), cv::Mat* debugImage = NULL);
};

#endif /* FIDUCIALDETECTOR_H_ */
