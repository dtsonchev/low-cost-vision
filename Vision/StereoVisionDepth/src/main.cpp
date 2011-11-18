#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include "stereovisiondepth.h"
#include <limits>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	if(argc < 2){
		std::cout << "to few arguments:\n"
			"   -c use two  camera for creating a depth view\n"
			"   -i use two images for creating a depth view"
		<< endl;
		exit(-1);
	}

	if(!(strcmp(argv[1], "-c"))){

		if(argc < 6){
			std::cout << "to few arguments:\n"
					"   -camL {value}   mandatory value: device number for the left camera\n"
					"   -camR {value}   mandatory value: device number for the right camera\n"
					"   -xmlD {value}   value: xml file with undistortion data"<< std::endl;
			exit(-1);
		}

		int allValues=0;
		cv::VideoCapture camL, camR;
		int camLnr = -1, camRnr =-1;
		Mat imgL, imgR;
		std::string undistorgenXMLpath = "";
		stereovisiondepth svd;
		Mat rmap[2][2];
		Rect validRoi[2];

		for(int i = 2; i < argc; i++){
			if(!(strcmp(argv[i], "-camL"))){
				camLnr = atoi(argv[i+1]);
				allValues = allValues | 1;
			}else if(!(strcmp(argv[i], "-camR"))){
				camRnr = atoi(argv[i+1]);
				allValues = allValues | 2;
			}else if(!(strcmp(argv[i], "-xmlD"))){
				undistorgenXMLpath = argv[i+1];
				allValues = allValues | 4;
			}
		}
		if(allValues !=3 && allValues != 7){
			cout<<"incorrect arguments";
			return -1;
		}
		camL = cv::VideoCapture(camLnr);
		camR = cv::VideoCapture(camRnr);

		if(!camL.isOpened() || !camR.isOpened()){
			cout<<"Camera not working corectly"<<endl;
			cout<<"\tCamnr Left: " << camLnr << "\n\tCamnr right: " << camRnr <<endl;
			exit(-1);
		}

		camL.read(imgL);
		camR.read(imgR);
		bool xmlUsed = false;
		const char* undistorgenXMLpathtemp = undistorgenXMLpath.c_str();

		if((strcmp(undistorgenXMLpathtemp,""))){
			FileStorage fs(undistorgenXMLpathtemp, CV_STORAGE_READ);
			Rect roiL;
			Rect roiR;
			if(fs.isOpened()){
				fs["RMAP00"] >> rmap[0][0];
				fs["RMAP01"] >> rmap[0][1];
				fs["RMAP10"] >> rmap[1][0];
				fs["RMAP11"] >> rmap[1][1];
				fs["ROI1_X"] >> validRoi[0].x;
				fs["ROI2_X"] >> validRoi[1].x;
				fs["ROI1_Y"] >> validRoi[0].y;
				fs["ROI2_Y"] >> validRoi[1].y;
				fs["ROI1_W"] >> validRoi[0].width;
				fs["ROI2_W"] >> validRoi[1].width;
				fs["ROI1_H"] >> validRoi[0].height;
				fs["ROI2_H"] >> validRoi[1].height;
				double sf = 600. / MAX(imgL.size().width, imgL.size().height);
				roiL = Rect(cvRound(validRoi[0].x * sf), cvRound(validRoi[0].y * sf),	cvRound( validRoi[0].width * sf), cvRound(validRoi[0].height * sf));
				roiR = Rect(cvRound(validRoi[1].x * sf), cvRound(validRoi[1].y * sf),	cvRound( validRoi[1].width * sf), cvRound(validRoi[1].height * sf));
				xmlUsed =true;
			}else{
				cout<<"Someting wrong with the xml";
				exit(-1);
			}
			fs.release();
			svd = stereovisiondepth(imgL, imgR,roiL,roiR);
		}else{
			svd= stereovisiondepth(imgL,imgR);
			xmlUsed = false;
		}

		Mat tempL,tempR;
		tempL = imgL.clone();
		tempR = imgR.clone();
		while((char)cv::waitKey(100) !='q'){
			camL.read(imgL);
			camR.read(imgR);

			if(xmlUsed){
				remap(imgL, tempL, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
				remap(imgR, tempR, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
				svd.updateImages(tempL,tempR);
			}else{
				svd.updateImages(imgL,imgR);
			}

			svd.DoBM();
			svd.DoSGBM();
			svd.DoHH();
			svd.DoVAR();
			svd.show();
		}
	}
	else if(!(strcmp(argv[1], "-i"))){

			if(argc < 6){
				std::cout << "to few arguments:\n"
						"   -imgL {value}   mandatory value: Path to the left image\n"
						"   -imgR {value}   mandatory value: Path to the right image\n"
						"   -xmlD {value}   value: xml file whit undistortion data\n"<< std::endl;
				exit(-1);
			}

			int allValues=0;
			Mat imgL, imgR;
			std::string imgPathLeft= "";
			std::string imgPathRight = "";
			std::string undistorgenXMLpath = "";
			stereovisiondepth svd;
			Mat rmap[2][2];
			Rect validRoi[2];

			for(int i = 2; i < argc; i++){
				if(!(strcmp(argv[i], "-imgL"))){
					imgPathLeft = argv[i+1];
					allValues = allValues | 1;
				}else if(!(strcmp(argv[i], "-imgR"))){
					imgPathRight = argv[i+1];
					allValues = allValues | 2;
				}else if(!(strcmp(argv[i], "-xmlD"))){
					undistorgenXMLpath = argv[i+1];
					allValues = allValues | 4;
				}
			}
			if(allValues !=3 && allValues != 7){
				cout<<"incorrect arguments";
				exit(-1);
			}
			imgL = cv::imread(imgPathLeft,-1);
			imgR = cv::imread(imgPathRight,-1);

			if(!imgL.data || !imgR.data){
				cout<<"img are not loaded  corectly"<<endl;
				cout<<"img path Left: \n\t" << imgPathLeft << "\nimg path right: \n\t" << imgPathRight <<endl;
				exit(-1);
			}
			const char* undistorgenXMLpathtemp = undistorgenXMLpath.c_str();
			if((strcmp(undistorgenXMLpathtemp,""))){
				FileStorage fs(undistorgenXMLpathtemp, CV_STORAGE_READ);
				Rect roiL;
				Rect roiR;
				if(fs.isOpened()){
					fs["RMAP00"] >> rmap[0][0];
					fs["RMAP01"] >> rmap[0][1];
					fs["RMAP10"] >> rmap[1][0];
					fs["RMAP11"] >> rmap[1][1];
					fs["ROI1_X"] >> validRoi[0].x;
					fs["ROI2_X"] >> validRoi[1].x;
					fs["ROI1_Y"] >> validRoi[0].y;
					fs["ROI2_Y"] >> validRoi[1].y;
					fs["ROI1_W"] >> validRoi[0].width;
					fs["ROI2_W"] >> validRoi[1].width;
					fs["ROI1_H"] >> validRoi[0].height;
					fs["ROI2_H"] >> validRoi[1].height;
					double sf = 600. / MAX(imgL.size().width, imgL.size().height);
					roiL = Rect(cvRound(validRoi[0].x * sf), cvRound(validRoi[0].y * sf),	cvRound( validRoi[0].width * sf), cvRound(validRoi[0].height * sf));
					roiR = Rect(cvRound(validRoi[1].x * sf), cvRound(validRoi[1].y * sf),	cvRound( validRoi[1].width * sf), cvRound(validRoi[1].height * sf));
					Mat tempL =imgL.clone();
					Mat tempR =imgR.clone();
					remap(tempL, imgL, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
					remap(tempR, imgR, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
				}else{
					cout<<"Someting wrong with the xml";
					exit(-1);
				}
				fs.release();
				svd = stereovisiondepth(imgL, imgR,roiL,roiR);
			}else{
				svd= stereovisiondepth(imgL,imgR);
			}
			while((char)cv::waitKey(100) !='q'){
				svd.DoBM();
				svd.DoSGBM();
				svd.DoHH();
				svd.DoVAR();
				svd.show();
			}
	}
	return 0;
}




