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
