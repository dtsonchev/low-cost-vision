//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DetectQRCode
// File:           DetectBarcode.cpp
// Description:    Detects barcodes and extract values
// Author:         Glenn Meerstra & Zep Mouris
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

#include "DetectQRCode/BarcodeDetector.h"
#include "DetectQRCode/MagickMat.h"
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

#include <Magick++.h>

int main (int argc, char **argv)
{
	if(argc>1){
		DetectBarcode db;
		cv::Mat image = cv::imread(argv[1]);
		std::string result;

		result = "BARCODE: ";
		if(db.detect(image, result)){
			std::cout << result << std::endl;
		}else{
			std::cout << "NOTHING FOUND" << std::endl;
		}
	}else{
		std::cout<<"giva a image"<<std::endl;
	}

    return 0;
}
