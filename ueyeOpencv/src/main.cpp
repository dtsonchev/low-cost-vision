#include <string>
#include <iostream>
#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <UEyeOpenCV.hpp>
using namespace std;
int main(int argc, char** argv) {

	UeyeOpencvCam cam1 = UeyeOpencvCam(0);
//	cam1.initCam();
	UeyeOpencvCam cam2 = UeyeOpencvCam(1);
//	cam2.initCam();
	while (true) {
		cv::namedWindow("cam1", CV_WINDOW_AUTOSIZE);
		cv::imshow("cam1", cam1.getFrame());

		cv::namedWindow("cam2", CV_WINDOW_AUTOSIZE);
		cv::imshow("cam2", cam2.getFrame());
		if (cv::waitKey(1) >= 0) {
			break;
		}
	}
	cam1.stopCam();
	cam2.stopCam();
////	cout << "is_SaveImage\t" << is_SaveImage(hCam, "/home/wouter/ueye.bmp") << endl;
	return 0;
}
