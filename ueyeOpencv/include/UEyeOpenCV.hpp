#pragma once
#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class UeyeOpencvCam {
public:
	UeyeOpencvCam(int id);
	void stopCam();
	cv::Mat getFrame();
private:
	HIDS hCam;
	cv::Mat mattie;
};
