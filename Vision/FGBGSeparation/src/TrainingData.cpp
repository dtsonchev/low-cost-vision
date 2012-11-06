//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        FGBGSeparation
// File:           TrainingData.cpp
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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <dirent.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <limits>
#include <FGBGSeparation/TrainingData.h>

using namespace cv;
using namespace std;

void Trainer::CreateTrainDataFromImage(Mat &image, Mat &binary, Mat &trainData, Mat &labels, int bins, int maskSize, int RGBorHSV){

	if(RGBorHSV == HSV){
		cvtColor(image, image, CV_RGB2HSV);
	}
	MatConstIterator_<Vec3b> it = image.begin<Vec3b>();
	MatConstIterator_<Vec3b> it_end = image.end<Vec3b>();
	MatConstIterator_<Vec3b> bit = binary.begin<Vec3b>();
	Vec3b pixBinary = *bit;
	//for every pixel
	for (; it != it_end; ++it, ++bit)
	{
		trainData.push_back(CreateHistogramFromPixel(it, bins, image.cols, image.rows, maskSize));
		pixBinary = *bit;
		if(pixBinary[0] > 127 && pixBinary[1] > 127 && pixBinary[2] > 127){
			labels.push_back(1);
		}else{
			labels.push_back(0);
		}
	}
}

Mat Trainer::CreateHistogramFromPixel(MatConstIterator_<Vec3b> it, int bins, int imageWidth, int imageHeight, int maskSize){
	float HSV[bins * 3];
	Vec3b pix;
	int pixelInMask = 0;
	MatConstIterator_<Vec3b> temp = it;

	for(int i = 0; i < (bins * 3); i++){
		HSV[i] = 0.0;
	}

	for(int h = -(maskSize/2); h <= maskSize/2; h++){
		for(int w = (maskSize/2) * -1; w <= maskSize/2; w++){

			if(!((it.pos().x + w) < 0 || (it.pos().x + w) > imageWidth || (it.pos().y + h) < 0 || (it.pos().y + h) > imageHeight)){

				pixelInMask++;
				pix = *(it + (imageWidth * h) + w);

				for(int i = 0; i < 3; i++){

					if(pix[i]>254){
						HSV[(int)((254/255.0)*bins)+(bins*i)]++;
					}else{
						HSV[(int)((pix[i]/255.0)*bins)+(bins*i)]++;
					}
				}
			}
		}
	}

	for(int i = 0; i < (bins * 3); i++){
		HSV[i] = HSV[i]/pixelInMask;
	}

	return Mat(1,bins*3,CV_32FC1, HSV);
}

