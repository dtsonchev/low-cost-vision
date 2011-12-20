//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        StereoVisionDepth
// File:           stereovisiondepth.h
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
#ifndef STEREOVISIONDEPTH_H_
#define STEREOVISIONDEPTH_H_

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

/**
 * @brief thic class creates 4 different depth images each with a different algorithm.
 */
class stereovisiondepth {
public:
	/**
	 *Useless constructor
	 */
	stereovisiondepth(){}
	/**
	 *Constructor used when no undistortion xml is used.
	 *@param imgL left image
	 *@param imgR right image
	 */
	stereovisiondepth(cv::Mat imgL, cv::Mat imgR);
	/**
	 *Constructor used when undistortion xml is used.
	 *@param imgL left image
	 *@param imgR right image
	 *@param roiL the region of interest of the left image.
	 *@param roiR the region of interest of the right image.
	 */
	stereovisiondepth(cv::Mat imgL, cv::Mat imgR,cv::Rect roiL,cv::Rect roiR);
	/**
	 * Destructor
	 */
	virtual ~stereovisiondepth(){}
	/**
	 * This function does the BM algorithm.
	 */
	void DoBM();
	/**
	 * This function does the SGBM algorithm.
	 */
	void DoSGBM();
	/**
	 * This function does the HH algorithm.
	 */
	void DoHH();
	/**
	 * This function does the VAR algorithm (currently no documentation at the opencv library)
	 */
	void DoVAR();
	/**
	 * updates the images used by the depth algorithms
	 * @param imgL The left images.
	 * @param imgR The right images.
	 */
	void updateImages(cv::Mat imgL, cv::Mat imgR);
	/**
	 * function which shows the result of the algorithms.
	 */
	void show();
	/**
	 * function for storing all the pixels separately.
	 * @param filename The path and name for the file.
	 */
	void saveXYZ(const char* filename);

	///BM algorithm implementation object.
	cv::StereoBM bm;
	///SGBM algorithm implementation object.
	cv::StereoSGBM sgbm;
	///HH algorithm implementation object.
	cv::StereoSGBM hh;
	///VAR algorithm implementation object, currently no documentation at opencv.
	cv::StereoVar var;

	///The left image used for all the algorithms
	cv::Mat imgL;
	///The right image used for all the algorithms
	cv::Mat imgR;

	///The result image with the results of all the algorithm
	cv::Mat resultBM;
	///The result image with the results of all the algorithm
	cv::Mat resultSGBM;
	///The result image with the results of all the algorithm
	cv::Mat resultHH;
	///The result image with the results of all the algorithm
	cv::Mat resultVAR;

	///a parameter of the BM algorithm.
	int bm_preFilterCapValue;
	///a parameter of the BM algorithm.
	int bm_SADWindowSize;
	///a parameter of the BM algorithm.
	int bm_minDisparity;
	///a parameter of the BM algorithm.
	int bm_numberOfDisparities;
	///a parameter of the BM algorithm.
	int bm_textureThreshold;
	///a parameter of the BM algorithm.
	int bm_uniquenessRatio;
	///a parameter of the BM algorithm.
	int bm_speckleWindowSize;
	///a parameter of the BM algorithm.
	int bm_speckleRange;
	///a parameter of the BM algorithm.
	int bm_disp12MaxDiff;


	/**A parameter of the SGBM algorithm.
	 * Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
	 */
	int sgbm_preFilterCap;
	/**A parameter of the SGBM algorithm.
	 * – Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
	 */
	int sgbm_SADWindowSize;
	/**A parameter of the SGBM algorithm.
	 * Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
	 */
	int sgbm_minDisparity;
	/**A parameter of the SGBM algorithm.
	 * Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16.
	 */
	int sgbm_numberOfDisparities;
	/**A parameter of the SGBM algorithm.
	 * Margin in percentage by which the best (minimum) computed cost function value should “win” the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
	 */
	int sgbm_uniquenessRatio;
	/**A parameter of the SGBM algorithm.
	 * Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
	 */
	int sgbm_speckleWindowSize;
	/**A parameter of the SGBM algorithm.
	 * Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, multiple of 16. Normally, 16 or 32 is good enough.
	 */
	int sgbm_speckleRange;
	/**A parameter of the SGBM algorithm.
	 * Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
	 */
	int sgbm_disp12MaxDiff;

	/**A parameter of the HH algorithm.
	 * Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
	 */
	int hh_preFilterCap;
	/**A parameter of the HH algorithm.
	 * – Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
	 */
	int hh_SADWindowSize;
	/**A parameter of the HH algorithm.
	 * Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
	 */
	int hh_minDisparity;
	/**A parameter of the HH algorithm.
	 * Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16.
	 */
	int hh_numberOfDisparities;
	/**A parameter of the HH algorithm.
	 * Margin in percentage by which the best (minimum) computed cost function value should “win” the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
	 */
	int hh_uniquenessRatio;
	/**A parameter of the HH algorithm.
	 * Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
	 */
	int hh_speckleWindowSize;
	/**A parameter of the HH algorithm.
	 * Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, multiple of 16. Normally, 16 or 32 is good enough.
	 */
	int hh_speckleRange;
	/**A parameter of the HH algorithm.
	 * Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
	 */
	int hh_disp12MaxDiff;

	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_levels;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_pyrScale ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_nIt ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_minDisp ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_maxDisp ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_poly_n ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_poly_sigma ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_fi ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_lambda ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_penalization ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_cycle ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).
	int var_flags ;
	///A parameter of the VAR algorithm.(currently no opencv documentation).

public:

	void initVar();
	void initTrackBars();
};

#endif /* STEREOVISIONDEPTH_H_ */
