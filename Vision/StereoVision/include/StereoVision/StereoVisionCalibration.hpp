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
