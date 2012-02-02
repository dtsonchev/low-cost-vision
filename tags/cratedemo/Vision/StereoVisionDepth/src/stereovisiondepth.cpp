//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        StereoVisionDepth
// File:           stereovisiondepth.cpp
// Description:    Program witch use 2 camera's for a gray image. Black means no data, white means very close and dark gray means far away. there are 4 types of depth algorithms
// Author:         Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of StereoVisionDepth.
//
// StereoVisionDepth is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// StereoVisionDepth is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with StereoVisionDepth.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include "stereovisiondepth.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

void updateBM(int, void* stereovisiondepthobject){
	stereovisiondepth* temp = (stereovisiondepth*) stereovisiondepthobject;

	temp->bm.state->preFilterCap = ((temp->bm_preFilterCapValue <= 0 ? 1 : temp->bm_preFilterCapValue)|1) ;
	temp->bm.state->SADWindowSize = ((temp->bm_SADWindowSize <= 4 ? 5 : temp->bm_SADWindowSize)|1);
	temp->bm.state->minDisparity = (temp->bm_minDisparity/16)*16;
	temp->bm.state->numberOfDisparities = ((temp->bm_numberOfDisparities <= 15 ? 16 : temp->bm_numberOfDisparities)/16)*16;
	temp->bm.state->textureThreshold = temp->bm_textureThreshold;
	temp->bm.state->uniquenessRatio = temp->bm_uniquenessRatio;
	temp->bm.state->speckleWindowSize = temp->bm_speckleWindowSize;
	temp->bm.state->speckleRange = temp->bm_speckleRange;
	temp->bm.state->disp12MaxDiff = temp->bm_disp12MaxDiff;
}

void updateSGBM(int, void* stereovisiondepthobject){
	stereovisiondepth* temp = (stereovisiondepth*) stereovisiondepthobject;
	temp->sgbm.preFilterCap = temp->sgbm_preFilterCap;
	temp->sgbm.SADWindowSize = ((temp->sgbm_SADWindowSize <= 0 ? 1 : temp->sgbm_SADWindowSize)|1);

	int cn = temp->imgL.channels();

	temp->sgbm.P1 = 8 * cn * temp->sgbm.SADWindowSize * temp->sgbm.SADWindowSize;
	temp->sgbm.P2 = 32 * cn * temp->sgbm.SADWindowSize * temp->sgbm.SADWindowSize;
	temp->sgbm.minDisparity = temp->sgbm_minDisparity;
	temp->sgbm.numberOfDisparities =  ((temp->sgbm_numberOfDisparities <= 15 ? 16 : temp->sgbm_numberOfDisparities)/16)*16;
	temp->sgbm.uniquenessRatio = temp->sgbm_uniquenessRatio;
	temp->sgbm.speckleWindowSize = temp->sgbm_speckleWindowSize;
	temp->sgbm.speckleRange = (temp->sgbm_speckleRange / 16) * 16;
	temp->sgbm.disp12MaxDiff = temp->sgbm_disp12MaxDiff;
}

void updateHH(int, void* stereovisiondepthobject){
	stereovisiondepth* temp = (stereovisiondepth*) stereovisiondepthobject;
	temp->hh.preFilterCap = temp->hh_preFilterCap;
	temp->hh.SADWindowSize = ((temp->hh_SADWindowSize <= 0 ? 1 : temp->hh_SADWindowSize)|1);

	int cn = temp->imgL.channels();

	temp->hh.P1 = 8 * cn * temp->hh.SADWindowSize * temp->hh.SADWindowSize;
	temp->hh.P2 = 32 * cn * temp->hh.SADWindowSize * temp->hh.SADWindowSize;
	temp->hh.minDisparity = temp->hh_minDisparity;

	temp->hh.numberOfDisparities =  ((temp->hh_numberOfDisparities <= 15 ? 16 : temp->hh_numberOfDisparities) / 16) * 16;
	temp->hh.uniquenessRatio = temp->hh_uniquenessRatio;
	temp->hh.speckleWindowSize = temp->hh_speckleWindowSize;
	temp->hh.speckleRange = (temp->hh_speckleRange / 16) * 16;
	temp->hh.disp12MaxDiff = temp->hh_disp12MaxDiff;
}

stereovisiondepth::stereovisiondepth(Mat imgL, Mat imgR) {
	this->imgL = imgL;
	this->imgR =imgR;

	resultBM = imgL.clone();
	resultSGBM = imgL.clone();
	resultHH = imgL.clone();
	resultVAR = imgL.clone();;

	imshow("bm", resultBM);
	imshow("sgbm", resultSGBM);
	imshow("hh", resultHH);
	imshow("var", resultVAR);
}

stereovisiondepth::stereovisiondepth(Mat imgL, Mat imgR, Rect roiL, Rect roiR) {
	this->imgL = imgL;
	this->imgR =imgR;

	bm.state->roi1 = roiL;
	bm.state->roi2 = roiR;

	resultBM = imgL.clone();
	resultSGBM = imgL.clone();
	resultHH = imgL.clone();
	resultVAR = imgL.clone();;

	imshow("bm", resultBM);
	imshow("sgbm", resultSGBM);
	imshow("hh", resultHH);
	imshow("var", resultVAR);
}

void stereovisiondepth::initTrackBars(){
     createTrackbar( "preFilterCap", "bm", &bm_preFilterCapValue, 63, updateBM, this );
	 createTrackbar( "SADWindowSize", "bm", &bm_SADWindowSize, 33 , updateBM, this);
	 createTrackbar( "minDisparity", "bm", &bm_minDisparity, 30 ,updateBM, this);
	 createTrackbar( "numberOfDisparities", "bm", &bm_numberOfDisparities, 255 ,updateBM, this);
	 createTrackbar( "textureThreshold", "bm", &bm_textureThreshold, 255,updateBM, this );
	 createTrackbar( "uniquenessRatio", "bm", &bm_uniquenessRatio, 30, updateBM, this);
	 createTrackbar( "speckleWindowSize", "bm", &bm_speckleWindowSize, 500 ,updateBM, this);
	 createTrackbar( "speckleRange", "bm", &bm_speckleRange, 10000,updateBM, this );
	 createTrackbar( "disp12MaxDiff", "bm", &bm_disp12MaxDiff, 10000 ,updateBM , this);

	 createTrackbar( "preFilterCap", "sgbm", &sgbm_preFilterCap, 255, updateSGBM , this);
	 createTrackbar( "SADWindowSize", "sgbm", &sgbm_SADWindowSize, 33, updateSGBM , this);
	 createTrackbar( "minDisparity", "sgbm", &sgbm_minDisparity, 30, updateSGBM , this);
	 createTrackbar( "numberOfDisparities", "sgbm", &sgbm_numberOfDisparities, 255, updateSGBM , this);
	 createTrackbar( "uniquenessRatio", "sgbm", &sgbm_uniquenessRatio, 50, updateSGBM , this);
	 createTrackbar( "speckleWindowSize", "sgbm", &sgbm_speckleWindowSize, 300, updateSGBM , this);
	 createTrackbar( "speckleRange", "sgbm", &sgbm_speckleRange, 2555, updateSGBM , this);
	 createTrackbar( "disp12MaxDiff", "sgbm", &sgbm_disp12MaxDiff, 50 ,updateSGBM , this);

	 createTrackbar( "preFilterCap", "hh", &hh_preFilterCap, 255, updateHH , this);
	 createTrackbar( "SADWindowSize", "hh", &hh_SADWindowSize, 33, updateHH , this);
	 createTrackbar( "minDisparity", "hh", &hh_minDisparity, 30, updateHH , this);
	 createTrackbar( "numberOfDisparities", "hh", &hh_numberOfDisparities, 255, updateHH , this);
	 createTrackbar( "uniquenessRatio", "hh", &hh_uniquenessRatio, 50, updateHH , this);
	 createTrackbar( "speckleWindowSize", "hh", &hh_speckleWindowSize, 300, updateHH , this);
	 createTrackbar( "speckleRange", "hh", &hh_speckleRange, 255, updateHH , this);
	 createTrackbar( "disp12MaxDiff", "hh", &hh_disp12MaxDiff, 50 ,updateHH , this);
}
void stereovisiondepth::initVar(){
	 //init on standaard values
	 bm_preFilterCapValue = 0;
	 bm_SADWindowSize = 0;
	 bm_minDisparity = 0;
	 bm_numberOfDisparities = 0;
	 bm_textureThreshold = 0;
	 bm_uniquenessRatio = 0;
	 bm_speckleWindowSize = 0;
	 bm_speckleRange = 0;
	 bm_disp12MaxDiff = 0;

	 sgbm_preFilterCap = 0;
	 sgbm_SADWindowSize = 0;
	 sgbm_minDisparity = 0;
	 sgbm_numberOfDisparities = 0;
	 sgbm_uniquenessRatio = 0;
	 sgbm_speckleWindowSize = 0;
	 sgbm_speckleRange = 0;
	 sgbm_disp12MaxDiff = 0;

	 hh_preFilterCap = 0;
	 hh_SADWindowSize = 0;
	 hh_minDisparity = 0;
	 hh_numberOfDisparities = 0;
	 hh_uniquenessRatio = 0;
	 hh_speckleWindowSize = 0;
	 hh_speckleRange = 0;
	 hh_disp12MaxDiff = 0;

	 var_levels = 0;							// ignored with USE_AUTO_PARAMS
	 var_pyrScale = 0.;							// ignored with USE_AUTO_PARAMS
	 var_nIt = 0;
	 var_minDisp = 0;
	 var_maxDisp = 0;
	 var_poly_n = 0;
	 var_poly_sigma = 0.0;
	 var_fi = 0.0;
	 var_lambda = 0.0;
	 var_penalization = var.PENALIZATION_TICHONOV;	// ignored with USE_AUTO_PARAMS
	 var_cycle = var.CYCLE_V;						// ignored with USE_AUTO_PARAMS
	 var_flags = 0;
}

void stereovisiondepth::DoBM(){
	Mat imgLL, imgRR, temp;
	cvtColor(imgL,imgLL, CV_RGB2GRAY);
	cvtColor(imgR, imgRR, CV_RGB2GRAY);
	bm(imgLL, imgRR, temp);
	temp.convertTo(resultBM, CV_8U, 255 / (bm.state->numberOfDisparities * 16.));
}

void stereovisiondepth::DoSGBM(){
	Mat temp;
	sgbm.fullDP = 0;
	sgbm.disp12MaxDiff = 1;
	sgbm.numberOfDisparities = (sgbm.numberOfDisparities == 0 ? 16 : sgbm.numberOfDisparities);
	sgbm(imgL, imgR, temp);
	temp.convertTo(resultSGBM, CV_8U, 255 / (sgbm.numberOfDisparities * 16.));
}

void stereovisiondepth::DoHH(){
	Mat temp;
	hh.fullDP = 1;
	hh.disp12MaxDiff = 1;
	hh.numberOfDisparities = (hh.numberOfDisparities == 0 ? 16 : hh.numberOfDisparities);
	hh(imgL, imgR, temp);

	temp.convertTo(resultHH, CV_8U, 255 / (hh.numberOfDisparities * 16.));
}

void stereovisiondepth::DoVAR(){
	Mat temp;
	var(imgL, imgR, temp);
	temp.convertTo(resultVAR, CV_8U);
}

void stereovisiondepth::updateImages(Mat imgL, Mat imgR){
	this->imgL = imgL;
	this->imgR = imgR;
}

void stereovisiondepth::show(){
	imshow("bm", resultBM);
	imshow("sgbm", resultSGBM);
	imshow("hh", resultHH);
	imshow("var", resultVAR);
}

void stereovisiondepth::saveXYZ(const char* filename)
{
	Mat matList[] = {resultBM, resultSGBM,resultHH,resultVAR};

	const double max_z = 1.0e4;
	FILE* fp = fopen(filename, "wt");

	for(int i = 0; i < 4; i++){
		for(int y = 0; y < matList[i].rows; y++){
			for(int x = 0; x < matList[i].cols; x++){

				Vec3f point = matList[i].at<Vec3f>(y, x);
				if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z){
					continue;
				}
				fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
			}
		}
		fprintf(fp, "\n\n\n\n");
    }
    fclose(fp);
}
