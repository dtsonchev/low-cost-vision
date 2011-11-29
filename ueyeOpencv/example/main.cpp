#include <string>
#include <iostream>
#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <UEyeOpenCV.hpp>
#include <UEyeOpenCVException.hpp>
using namespace std;
int main(int argc, char** argv) {
try{
	UeyeOpencvCam cam1 = UeyeOpencvCam();
	UeyeOpencvCam cam2 = UeyeOpencvCam();
	while (true) {
		cv::imshow("cam1", cam1.getFrame());

		cv::namedWindow("cam2", CV_WINDOW_AUTOSIZE);
		cv::imshow("cam2", cam2.getFrame());
		if (cv::waitKey(1) >= 0) {
			break;
		}
	}
}
catch(UeyeOpenCVException& e){
	cout << e.what();
}

	return 0;
}
