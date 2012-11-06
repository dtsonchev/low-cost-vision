//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DifferentsesSeparation
// File:           liblocator.h
// Description:    A header for marking differnces between images
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
#ifndef LOCATOR_H
#define LOCATOR_H


/**
* @brief "A class which can use for detect objects different from te background"
* @author Glenn Meerstra
* @author Zep Mouris
* @version 2.0
* @date 18-10-2011
*/

#include <opencv2/highgui/highgui.hpp>
#include <Locator/cameraException.h>

//Locator class
class Locator{
public:
	/**
	 * @brief constructor
	 * @param device device number of the camera
	 */
	Locator(int device);

	/**
	 * @brief deconstructor
	 */
	~Locator();
		
	/**
	 * @brief function for setting the background. this background is used for detect the differences.
	 * @param stableBackground Mat object which contains the background after calling the function.
	 */
	void setBackground(cv::Mat &stableBackground);
    
	/**
	 * @brief this function wait for any change in the view and after this it wait til the is stable again.
	 * @param stableImage Mat object which contains the new stable background after calling the function.
	 */
	void WaitForStableViewAndTakeImage(cv::Mat &stableImage);
	
    /**
	 * @brief this function shows the differents between the stable image and the stable background.
	 * @param difference Mat object which contains the back and white image after calling the function. back means not changed and white means changed.
	 */
	void showDifference( cv::Mat &difference);
    
	/**
	 * @brief this function draw a rectangle on the image for every blob.
	 * @param Blobs Mat object which contains a image whit te blobs after calling the function.
	 */
	void findAndDrawBlobs(cv::Mat &Blobs );


	/**
	 * @brief this function draw a rectangle on the image for every blob.
	 * @param -
	 */
	inline void getObjectLocations(std::vector<cv::Rect> &objectLocations );

	///@brief contours smaller as this number will be ignored
	char minContourSize;
	///@brief the number which describe the range for a image which is the same or not
	char normRange;
	///@brief The number of steps on a R,G,B channel before it is different.
	char differents;
	///@brief the number for the iterations for dilate and erode.
	char filterIterations;

private:
	void setStdVars(){
		 minContourSize = 40;
		 normRange = 30;
		 differents = 30;
		 filterIterations = 3;
		 maxNorm = 0;		
	}
	
	cv::VideoCapture cam;
	double maxNorm;
	double norm;
	cv::Mat lastCapturedFrame;
	cv::Mat lastStableBackground;
	cv::Mat matchingBackground;
	cv::Mat detectedDifference;
	
	int internalKey;

	std::vector<cv::Rect> objectLocations;
};

void Locator::getObjectLocations(std::vector<cv::Rect> &Locations ){
	Locations = objectLocations;
}

#endif
