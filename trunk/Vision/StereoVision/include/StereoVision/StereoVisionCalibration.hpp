//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        StereoVision
// File:           StereoVisionCalibration.hpp
// Description:    This class creates and uses a matrix in order to rectify an image the matrix is created by loading multiple sets of images with checker board pattern
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
#include <opencv2/core/core.hpp>

namespace stereoVision {

/**
 * @brief This class creates and uses a matrix in order to rectify an image
 * the matrix is created by loading multiple sets of images with checker board pattern
 */
class StereoVisionCalibration {
private:
	///The matrixes used for re-mapping the images
	cv::Mat rmap[2][2];

public:
    ///The valid regions within the two correctified images
	cv::Rect validRoi[2];

    /**
     * Creates a xml file for correcting the images
     * @param imagelist a list with all the file names
     * @param xmlName the name of the resulting xml file
     * @param boardSize The size of the checker board (the amount of horizontal squares -1 , the amount of vertical squares -1)
     * @return <b>false</b> if it didn't succeed
     */
	bool StereoCalib(const std::string& filename, const std::string& xmlName, cv::Size boardSize);
	bool StereoCalib(const std::vector<cv::Mat>& imagelist, const std::string& xmlName, cv::Size boardSize);
    /**
     * Reads the matrixes for rectifying the images
     */
	bool initRectifyImage(const std::string& xmlName);
	/**
	 * The function rectifies two images
	 * @param inputL the left input image
	 * @param outputL the left corrected output image
	 * @param inputR the right input image
	 * @param outputR the right corrected output image
	 * @param drawOnImage when true a box around the region of interest is drawn on the output image default value is <b>false</b>
	 */
	void RectifyImage(const cv::Mat inputL, cv::Mat &outputL,
			const cv::Mat inputR, cv::Mat &outputR, bool drawOnImage = false);
};

}
