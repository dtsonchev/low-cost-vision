#pragma once

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

namespace stereoVision {

/**
 * @brief class to obtain depth information from two 2D images
 */
class StereoVision {
public:
	/**
	 * Constructor used when undistortion xml is used.
	 */

	StereoVision(cv::Size sz, std::string xmlPath);
	/**
	 * Destructor
	 */
	virtual ~StereoVision(){}
	/**
	 * Resets all variables.
	 */
	void resetVariables();
	/**
	 * Show (and add) trackbars that set the stereo algorithm parameters on a window
	 * @param windowName the name of the window
	 */
	void showTrackBars(std::string windowName);
	/**
	 * This function rectifies both images and runs the Semi-Global Block Matching algorithm (by Ph. D., M. Sc. Dipl.-Inform Heiko Hirschmüller)
	 * @param leftImage
	 * @param rightImage
	 * @return a greyscale image with depth information
	 */
	cv::Mat createDepthImage(cv::Mat& leftImage, cv::Mat& rightImage);
	/**
	 * Call-back function for the trackbars
	 * @param first parameters is not used
	 * @param stereoVisionObject pointer to self
	 */
	friend void updateSGBM(int, void* stereoVisionObject);
private:
	///SGBM algorithm implementation object.
	cv::StereoSGBM sgbm;

	///The left image used for all the algorithms
	cv::Mat imgL;
	///The right image used for all the algorithms
	cv::Mat imgR;

	///The result image with the results of all the algorithm
	cv::Mat resultSGBM;
	///Rectifying image matrices
	cv::Mat rmap[2][2];
	///Regions of the images that should be used
	cv::Rect roiL, roiR;
};

}
