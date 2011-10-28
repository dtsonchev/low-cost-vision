#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <dirent.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <limits>
#include "TrainingData.h"

using namespace cv;
using namespace std;

void Trainer::CreateTrainDataFromImage(Mat &image, Mat &binary, Mat &trainData, Mat &labels, int bins, int maskSize, int RGBorHSV){

	if(RGBorHSV == HSV){
		cvtColor(image, image, CV_RGB2HSV);
	}
	MatConstIterator_<Vec3b> it = image.begin<Vec3b>();
	MatConstIterator_<Vec3b> it_end = image.end<Vec3b>();
	MatConstIterator_<Vec3b> bit = binary.begin<Vec3b>();
	Vec3b pixBinary = *bit;
	//for every pixel
	for (; it != it_end; ++it, ++bit)
	{
		trainData.push_back(CreateHistogramFromPixel(it, bins, image.cols, image.rows, maskSize));
		pixBinary = *bit;
		if(pixBinary[0] > 127 && pixBinary[1] > 127 && pixBinary[2] > 127){
			labels.push_back(1);
		}else{
			labels.push_back(0);
		}
	}
}

Mat Trainer::CreateHistogramFromPixel(MatConstIterator_<Vec3b> it, int bins, int imageWidth, int imageHeight, int maskSize){
	float HSV[bins * 3];
	Vec3b pix;
	int pixelInMask = 0;
	MatConstIterator_<Vec3b> temp = it;

	for(int i = 0; i < (bins * 3); i++){
		HSV[i] = 0.0;
	}

	for(int h = -(maskSize/2); h <= maskSize/2; h++){
		for(int w = (maskSize/2) * -1; w <= maskSize/2; w++){

			if(!((it.pos().x + w) < 0 || (it.pos().x + w) > imageWidth || (it.pos().y + h) < 0 || (it.pos().y + h) > imageHeight)){

				pixelInMask++;
				pix = *(it + (imageWidth * h) + w);

				for(int i = 0; i < 3; i++){

					if(pix[i]>254){
						HSV[(int)((254/255.0)*bins)+(bins*i)]++;
					}else{
						HSV[(int)((pix[i]/255.0)*bins)+(bins*i)]++;
					}
				}
			}
		}
	}

	for(int i = 0; i < (bins * 3); i++){
		HSV[i] = HSV[i]/pixelInMask;
	}

	return Mat(1,bins*3,CV_32FC1, HSV);
}

