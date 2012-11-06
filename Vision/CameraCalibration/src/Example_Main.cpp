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
