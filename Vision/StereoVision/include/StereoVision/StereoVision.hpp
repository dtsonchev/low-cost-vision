//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        StereoVision
// File:           StereoVision.hpp
// Description:    class to obtain depth information from two 2D images
// Author:         Franc Pape & Wouter Langerak
// Notes:          
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

#pragma once

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

namespace stereoVision {

/**
 * @brief class to obtain depth information from two 2D images
 */
class StereoVision {
public:
	/**
	 * Constructor used when undistortion xml is used.
	 */

	StereoVision(cv::Size sz, std::string xmlPath);
	/**
	 * Destructor
	 */
	virtual ~StereoVision(){}
	/**
	 * Resets all variables.
	 */
	void resetVariables();
	/**
	 * Show (and add) trackbars that set the stereo algorithm parameters on a window
	 * @param windowName the name of the window
	 */
	void showTrackBars(std::string windowName);
	/**
	 * This function rectifies both images and runs the Semi-Global Block Matching algorithm (by Ph. D., M. Sc. Dipl.-Inform Heiko Hirschmüller)
	 * @param leftImage
	 * @param rightImage
	 * @return a greyscale image with depth information
	 */
	cv::Mat createDepthImage(cv::Mat& leftImage, cv::Mat& rightImage);
	/**
	 * Call-back function for the trackbars
	 * @param first parameters is not used
	 * @param stereoVisionObject pointer to self
	 */
	friend void updateSGBM(int, void* stereoVisionObject);
private:
	///SGBM algorithm implementation object.
	cv::StereoSGBM sgbm;

	///The left image used for all the algorithms
	cv::Mat imgL;
	///The right image used for all the algorithms
	cv::Mat imgR;

	///The result image with the results of all the algorithm
	cv::Mat resultSGBM;
	///Rectifying image matrices
	cv::Mat rmap[2][2];
	///Regions of the images that should be used
	cv::Rect roiL, roiR;
};

}
