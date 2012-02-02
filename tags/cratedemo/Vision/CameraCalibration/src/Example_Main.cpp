//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CameraCalibration
// File:           Example_Main.cpp
// Description:    An example in how to use this library
// Author:         Glenn Meerstra & Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of CameraCalibration.
//
// CameraCalibration is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// CameraCalibration is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with CameraCalibration.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#include <CameraCalibration/RectifyImage.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]){

	string command;
	if(argc < 2 ){
		cout << "First argument has to be train or correct" << endl;
		return -1;
	}else{
		command = argv[1];
		if(command != "train" && command != "correct"){
			cout << "First argument has to be train or correct" << endl;
			return -1;
		}
	}

	RectifyImage ri;
	if(command == "train"){
		if(argc < 4){
			cout << "second argument has to be the image directory, the third the xml name" << endl;
			return -1;
		}

		cv::Size boardSize(9,6);
		if(ri.createXML(argv[2], boardSize, argv[3])<=0){
			cout << "training failed" << endl;
			return -1;
		}
		cout << "DONE training" << endl;
	}else if (command == "correct"){
		if(argc < 4){
			cout << "second argument has to be the xml name, the third an image to rectify" << endl;
			return -1;
		}


		cv::Mat image = cv::imread(argv[3]);
		if(!ri.initRectify(argv[2], cv::Size(image.cols, image.rows))){
			cout << "XML not found" << endl;
			return -1;
		}
		cv::imshow("Original", image);
		cvMoveWindow("Original", 0, 100);
		cv::Mat temp = image.clone();
		ri.rectify(image, temp);
		cv::imshow("Corrected", temp);
		cvMoveWindow("Corrected", image.cols, 100);

		while(1){
			if( cvWaitKey (100) == 'q' ) break;
		}

		image.release();
		cv::destroyWindow("Corrected");
		cv::destroyWindow("Original");

		cout << "DONE correcting" << endl;
	}
	return 0;
}
