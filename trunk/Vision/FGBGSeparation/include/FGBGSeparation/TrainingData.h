//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        FGBGSeparation
// File:           TrainingData.h
// Description:    library can be trained with images. with normal images and FGBG images where white means forground and black means background after training the library can generate the FGBG images self. this class generats the raw data
// Author:         Glenn Meerstra & Zep Mouris
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
#ifndef TRAINER_H_
#define TRAINER_H_

#include <opencv2/core/core.hpp>

typedef enum RGBorHSVenum{
	Neither,
	RGB,
	HSV
} RGBorHSVenum;

class Trainer{
public:

	/**
	 * @brief creates an histograms foreach pixel and decides if it's fore- or background
	 * @param image the GRB/HSV image
	 * @param binary a black and white image to differentiate the fore- and background
	 * @param trainData an matrix were the the histograms are placed in
	 * @param labels for each matrix is there an label put into this matrix
	 * @param bins the amount of bins where the values a pixel within an image
	 * @param maskSize the size of the mask at which the histograms are made
	 * @param RGBorHSV 1 for RGB and 2 for HSV
	 * @note an label is fore- or background
	 */
	void CreateTrainDataFromImage(cv::Mat &image, cv::Mat &binary, cv::Mat &trainData, cv::Mat &labels, int bins, int maskSize, int RGBorHSV);
    
	/**
	 * @brief creates an histogram with the size of maskSize at the pixel were it points to
	 * @param it points to an pixel within an image
	 * @param bins the amount of bins where the values are divided over
	 * @param imageWidth the width of an image
	 * @param imageHeight the height of an image
	 * @param maskSize the size of the mask at which the histograms are made
	 * @return the created histogram
	 */
	cv::Mat CreateHistogramFromPixel(cv::MatConstIterator_<cv::Vec3b> it, int bins, int imageWidth, int imageHeight, int maskSize);
};

#endif /*TRAINER_H_*/
