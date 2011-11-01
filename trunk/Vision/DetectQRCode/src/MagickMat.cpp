#include "MagickMat.h"
#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>

bool MagickMatConverter::Magick2Mat(Magick::Image &magickImage, cv::Mat &matImage){
	magickImage.write("Temp.bmp");
	if(boost::filesystem::exists("Temp.bmp")){
		matImage = cv::imread("Temp.bmp");
		boost::filesystem::remove("Temp.bmp");
		return true;
	}else{
		return false;
	}
}

bool MagickMatConverter::Mat2Magick(const cv::Mat &matImage, Magick::Image &magickImage){
	cv::imwrite("Temp.bmp", matImage);
	if(boost::filesystem::exists("Temp.bmp")){
		magickImage.read("Temp.bmp");
		boost::filesystem::remove("Temp.bmp");
		return true;
	}else{
		return false;
	}
}
