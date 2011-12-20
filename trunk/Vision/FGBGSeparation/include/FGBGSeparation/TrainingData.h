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
// License:        GNU GPL v3
//
// This file is part of FGBGSeparation.
//
// FGBGSeparation is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// FGBGSeparation is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with FGBGSeparation.  If not, see <http://www.gnu.org/licenses/>.
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
