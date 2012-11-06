//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DifferentsesSeparation
// File:           Main.cpp
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


#include <iostream>
#include <sstream>
#include <cstdio>
#include <Locator/liblocator.h>
#include <Locator/cameraException.h>

using namespace std;

/*! 
 * The main function opens an connection with a webcam on video1
 * by pressing the 'b' key you take an background image
 * when an new object is introduced in the webcams line of sight the
 * detected object is calculated when the image is stable.
 * by pressing 'q' you exit the program.
 */
int main(int argc, char** argv){
	try{				
		int deviceNr = 0;
		if(argc > 1){
			stringstream s;
			s << argv[1];
			s >> deviceNr;
		}
		Locator loca(deviceNr);
		int key = 0;
		cv::Mat Background;

		std::cout << "Waiting on b(ackground) || q(uit)" << std::endl;
		while (key != 'b' && key != 'q'){
			key = cv::waitKey(100);
			loca.setBackground(Background);
			cv::imshow("background",Background);
		}
		cv::Mat view, difference, blobs;

		blobs = Background.clone();
		difference = Background.clone();
		cv::namedWindow("blobs");
		cvMoveWindow("blobs", blobs.cols + 100, 200);
		cv::namedWindow("differences");
		cvMoveWindow("differences", 100, 200);

		std::cout << "Press q to quit" << std::endl;
		while(key != 'q'){
			loca.WaitForStableViewAndTakeImage(view);
			loca.showDifference(difference);
			loca.findAndDrawBlobs(blobs );
			imshow("blobs",blobs);								
			imshow("differences",difference);	
			key = cv::waitKey(100);
		}
	    
	} catch(exception &e){
		cout << e.what() << endl;
	}

	return 0;
}
