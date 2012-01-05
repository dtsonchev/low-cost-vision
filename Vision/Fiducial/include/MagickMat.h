//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DetectQRCode
// File:           MagickMat.h
// Description:    Converts opencv Mat objects and Magick Image objects
// Author:         Glenn Meerstra
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of DetectQRCode.
//
// DetectQRCode is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DetectQRCode is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with DetectQRCode.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#ifndef MAGICKMAT_H_
#define MAGICKMAT_H_

#include <opencv2/core/core.hpp>
#include <Magick++.h>
/**
 * @brief this class can convert an opencv Mat object into a magick image or \n
 * converts a magick image into an opencv Mat object
 */
class MagickMatConverter{
public:
    ///@brief constructor
	MagickMatConverter(){}
    ///@brief deconstructor
	~MagickMatConverter(){}

	/**
	 * @fn Magick2Mat(Magick::Image &magickImage, cv::Mat &matImage)
	 * @brief converts a magick image into an opencv mat
	 * @param magickImage the src image
	 * @param matImage the image to convert the src to
	 * @return true if it succeeded
	 */
	bool Magick2Mat(Magick::Image &magickImage, cv::Mat &matImage);
	/**
	 * @fn Mat2Magick(const cv::Mat &matImage, Magick::Image &magickImage)
	 * @brief converts an opencv mat to a magick image
	 * @param matImage the src image
	 * @param magickImage the image to convert the src to
	 * @return
	 */
	bool Mat2Magick(const cv::Mat &matImage, Magick::Image &magickImage);
};


#endif /* MAGICKMAT_H_ */
