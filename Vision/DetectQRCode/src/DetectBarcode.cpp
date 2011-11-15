#include "BarcodeDetector.h"
#include "MagickMat.h"
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

#include <Magick++.h>

int main (int argc, char **argv)
{
	if(argc>1){
		DetectBarcode db;
		cv::Mat image = cv::imread(argv[1]);
		std::string result;

		result = "BARCODE: ";
		if(db.detect(image, result)){
			std::cout << result << std::endl;
		}else{
			std::cout << "NOTHING FOUND" << std::endl;
		}
	}else{
		std::cout<<"giva a image"<<std::endl;
	}

    return 0;
}
