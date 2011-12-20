//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DetectQRCode
// File:           MagickMat.cpp
// Description:    Converts opencv Mat objects and Magick image objects
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

#include "DetectQRCode/MagickMat.h"
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>

bool MagickMatConverter::Magick2Mat(Magick::Image &magickImage, cv::Mat &matImage){
	magickImage.write("Temp.bmp");
	if(boost::filesystem::exists("Temp.bmp")){
		matImage = cv::imread("Temp.bmp");
		boost::filesystem::remove("Temp.bmp");
		return true;
	}else{
		return false;
	}
}

bool MagickMatConverter::Mat2Magick(const cv::Mat &matImage, Magick::Image &magickImage){
	cv::imwrite("Temp.bmp", matImage);
	if(boost::filesystem::exists("Temp.bmp")){
		magickImage.read("Temp.bmp");
		boost::filesystem::remove("Temp.bmp");
		return true;
	}else{
		return false;
	}
}
