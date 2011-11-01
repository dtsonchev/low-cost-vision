#include "RectifyImage.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]){

	if(argc < 2){
		cout << "First argument has to be train or correct" << endl;
		return -1;
	}

	string command = argv[1];
	RectifyImage ri;
	if(command == "train"){
		if(argc < 4){
			cout << "second argument has to be the image directory, the third the xml name" << endl;
			return -1;
		}

		cv::Size boardSize(9,6);
		if(!ri.createXML(argv[2], boardSize, argv[3])){
			return -1;
		}
		cout << "DONE training" << endl;
	}else if (command == "correct"){
		if(argc < 4){
			cout << "second argument has to be the xml name, the third an image to rectify" << endl;
			return -1;
		}


		cv::Mat image = cv::imread(argv[3]);
		if(ri.initRectify(argv[2], cv::Size(image.cols, image.rows)) < 0){
			cout << "XML not found" << endl;
			return -1;
		}
		cv::imshow("Original", image);
		cvMoveWindow("Original", 0, 100);
		image = ri.rectify(image);
		cv::imshow("Corrected", image);
		cvMoveWindow("Corrected", image.cols, 100);

		while(1){
			if( cvWaitKey (100) == 'q' ) break;
		}

		image.release();
		cv::destroyWindow("Corrected");
		cv::destroyWindow("Original");

		cout << "DONE correcting" << endl;
	}
	return 0;
}
