#pragma once
#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <UEyeOpenCVException.hpp>
class UeyeOpencvCam {
public:
	/**
	 *
	 * @param id
	 */
	UeyeOpencvCam();
	HIDS getHIDS();
	void stopCam();
	~UeyeOpencvCam();
	cv::Mat getFrame();
	void setAutoWhiteBalance(bool set=true);
private:
	HIDS hCam;
	cv::Mat mattie;
};
