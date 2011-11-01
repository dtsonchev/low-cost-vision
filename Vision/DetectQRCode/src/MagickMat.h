#ifndef MAGICKMAT_H_
#define MAGICKMAT_H_

#include <opencv2/core/core.hpp>
#include <Magick++.h>
/**
 * @brief this class can convert an opencv Mat object into a magick image or \n
 * converts a magick image into an opencv Mat object
 */
class MagickMatConverter{
public:
    ///@brief constructor
	MagickMatConverter(){}
    ///@brief deconstructor
	~MagickMatConverter(){}

	/**
	 * @fn Magick2Mat(Magick::Image &magickImage, cv::Mat &matImage)
	 * @brief converts a magick image into an opencv mat
	 * @param magickImage the src image
	 * @param matImage the image to convert the src to
	 * @return true if it succeeded
	 */
	bool Magick2Mat(Magick::Image &magickImage, cv::Mat &matImage);
	/**
	 * @fn Mat2Magick(const cv::Mat &matImage, Magick::Image &magickImage)
	 * @brief converts an opencv mat to a magick image
	 * @param matImage the src image
	 * @param magickImage the image to convert the src to
	 * @return
	 */
	bool Mat2Magick(const cv::Mat &matImage, Magick::Image &magickImage);
};


#endif /* MAGICKMAT_H_ */
