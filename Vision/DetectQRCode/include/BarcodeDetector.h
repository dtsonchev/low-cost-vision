#ifndef DETECTBARCODE_H_
#define DETECTBARCODE_H_

#include <stdlib.h>
#include <zbar.h>
#include <iostream>
#include "MagickMat.h"
#include <Magick++.h>
#include <opencv2/core/core.hpp>

/**
 * @brief This class can detect barcodes from a Mat object
 */
class DetectBarcode{
private:
	///@brief the scanner which scans the code from an image
    zbar::ImageScanner scanner;
	MagickMatConverter converter;

public:
    ///@brief constructor sets the values for the scanner
    DetectBarcode();
    ///@brief deconstructor
    ~DetectBarcode();

    /**
     * @fn bool detect(cv::Mat image, std::string &result)
     * @brief detects codes on the image
     * @param image the image to detect the code on
     * @param result the string to write the result to
     * @return true if we have a result
     */
	bool detect(cv::Mat image, std::string &result);
};


#endif /* DETECTBARCODE_H_ */
