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
