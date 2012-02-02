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
// License:        GNU GPL v3
//
// This file is part of StereoVision.
//
// StereoVision is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// StereoVision is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with StereoVision.  If not, see <http://www.gnu.org/licenses/>.
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
