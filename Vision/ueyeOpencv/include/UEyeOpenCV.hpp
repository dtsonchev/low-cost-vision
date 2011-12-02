#pragma once
#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <UEyeOpenCVException.hpp>
class UeyeOpencvCam {
public:
        UeyeOpencvCam(int wdth, int heigh);
        HIDS getHIDS();
        void stopCam();
        ~UeyeOpencvCam();
        cv::Mat getFrame();
        void getFrame(cv::Mat& mat);
        void setAutoWhiteBalance(bool set=true);
        void setAutoGain(bool set=true);
private:
        HIDS hCam;
        cv::Mat mattie;
        int width;
        int height;
};
