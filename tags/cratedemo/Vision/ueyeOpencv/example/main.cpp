//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        ueyeOpencv
// File:           main.cpp
// Description:    Example for UeyeOpenCV
// Author:         Wouter Langerak
// Notes:          For more functionalities use the SDK of UEye, the purpose of this project is to make it compatible with OpenCV Mat.
//
// License:        GNU GPL v3
//
// This file is part of ueyeOpencv.
//
// ueyeOpencv is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// ueyeOpencv is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with ueyeOpencv.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <string>
#include <iostream>
#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <UEyeOpenCV.hpp>
#include <UEyeOpenCVException.hpp>
using namespace std;
int main(int argc, char** argv) {
try{
	UeyeOpencvCam cam1 = UeyeOpencvCam(640,480);
	UeyeOpencvCam cam2 = UeyeOpencvCam(640,480);


	while (true) {
		cv::imshow("cam1", cam1.getFrame());

		cv::namedWindow("cam2", CV_WINDOW_AUTOSIZE);
		cv::imshow("cam2", cam2.getFrame());
		if (cv::waitKey(1) >= 0) {
			break;
		}
	}
}
catch(UeyeOpenCVException& e){
	cout << e.what();
}

	return 0;
}
