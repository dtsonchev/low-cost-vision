#ifndef TRAINER_H_
#define TRAINER_H_

#include <opencv2/core/core.hpp>

typedef enum RGBorHSVenum{
	Neither,
	RGB,
	HSV
} RGBorHSVenum;

using namespace cv;

/**
 * @brief An class were histograms are created for machine learning
 * @author Zep
 * @author Glenn
 * @version 2.0
 * @date 10-2011
 */
class Trainer{
public:

	/**
	 * @brief creates an histograms foreach pixel and decides if it's fore- or background
	 * @param image the GRB/HSV image
	 * @param binary a black and white image to differentiate the fore- and background
	 * @param trainData an matrix were the the histograms are placed in
	 * @param labels for each matrix is there an label put into this matrix
	 * @param bins the amount of bins where the values a pixel within an image
	 * @param maskSize the size of the mask at which the histograms are made
	 * @param RGBorHSV 1 for RGB and 2 for HSV
	 * @note an label is fore- or background
	 */
	void CreateTrainDataFromImage(Mat &image, Mat &binary, Mat &trainData, Mat &labels, int bins, int maskSize, int RGBorHSV);
    
	/**
	 * @brief creates an histogram with the size of maskSize at the pixel were it points to
	 * @param it points to an pixel within an image
	 * @param bins the amount of bins where the values are divided over
	 * @param imageWidth the width of an image
	 * @param imageHeight the height of an image
	 * @param maskSize the size of the mask at which the histograms are made
	 * @return the created histogram
	 */
	Mat CreateHistogramFromPixel(MatConstIterator_<Vec3b> it, int bins, int imageWidth, int imageHeight, int maskSize);
};

#endif /*TRAINER_H_*/
