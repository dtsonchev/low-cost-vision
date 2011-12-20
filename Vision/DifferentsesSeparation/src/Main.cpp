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
// License:        GNU GPL v3
//
// This file is part of DifferentsesSeparation.
//
// DifferentsesSeparation is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DifferentsesSeparation is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with DifferentsesSeparation.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#include <iostream>
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
		Locator loca(0);
		int key = 0;
		cv::Mat Background;

		std::cout << "Waiting on b(ackground) || q(uit)" << std::endl;
		while (key != 'b' && key != 'q'){
			key = cv::waitKey(100);
			loca.setBackground(Background);
			cv::imshow("background",Background);
		}
		cv::Mat view, difference, blobs;

		std::cout << "Press q to quit" << std::endl;
		while(key != 'q'){
			loca.WaitForStableViewAndTakeImage(view);
			loca.showDifference(difference);
			loca.findAndDrawBlobs(blobs );
			imshow("blobs",blobs);	
			key = cv::waitKey(100);
		}
	    
	} catch(exception &e){
		cout << e.what() << endl;
	}

	return 0;
}
