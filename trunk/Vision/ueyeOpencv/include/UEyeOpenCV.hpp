#pragma once
#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <UEyeOpenCVException.hpp>
//TODO: create function to enable/disable auto-gain
class UeyeOpencvCam {
public:
	UeyeOpencvCam();
	HIDS getHIDS();
	void stopCam();
	~UeyeOpencvCam();
	/// @deprecated use getFrame(cv::Mat&):void
	cv::Mat getFrame();
	void getFrame(cv::Mat& mat);
	void setAutoWhiteBalance(bool set=true);
private:
	HIDS hCam;
	cv::Mat mattie;
};
