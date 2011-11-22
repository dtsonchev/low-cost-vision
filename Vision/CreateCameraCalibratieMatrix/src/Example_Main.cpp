#include <CameraCalibration/RectifyImage.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]){

	string command;
	if(argc < 2 ){
		cout << "First argument has to be train or correct" << endl;
		return -1;
	}else{
		command = argv[1];
		if(command != "train" && command != "correct"){
			cout << "First argument has to be train or correct" << endl;
			return -1;
		}
	}

	RectifyImage ri;
	if(command == "train"){
		if(argc < 4){
			cout << "second argument has to be the image directory, the third the xml name" << endl;
			return -1;
		}

		cv::Size boardSize(9,6);
		if(ri.createXML(argv[2], boardSize, argv[3])<=0){
			cout << "training failed" << endl;
			return -1;
		}
		cout << "DONE training" << endl;
	}else if (command == "correct"){
		if(argc < 4){
			cout << "second argument has to be the xml name, the third an image to rectify" << endl;
			return -1;
		}


		cv::Mat image = cv::imread(argv[3]);
		if(!ri.initRectify(argv[2], cv::Size(image.cols, image.rows))){
			cout << "XML not found" << endl;
			return -1;
		}
		cv::imshow("Original", image);
		cvMoveWindow("Original", 0, 100);
		cv::Mat temp = image.clone();
		ri.rectify(image, temp);
		cv::imshow("Corrected", temp);
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
