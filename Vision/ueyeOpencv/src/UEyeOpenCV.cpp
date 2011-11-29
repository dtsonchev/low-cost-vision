#include <UEyeOpenCV.hpp>
#include <iostream>
#include <ueye.h>

UeyeOpencvCam::UeyeOpencvCam() {
	using std::cout;
	using std::endl;
	mattie = cv::Mat(480, 640, CV_8UC3);
	hCam = 0;
	char* ppcImgMem;
	int pid;
	INT nAOISupported = 0;
	double on = 1;
	double empty;
	int retInt = IS_SUCCESS;
	retInt = is_InitCamera(&hCam, 0);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
	retInt = is_SetColorMode(hCam, IS_CM_BGR8_PACKED);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
	retInt = is_ImageFormat(hCam, IMGFRMT_CMD_GET_ARBITRARY_AOI_SUPPORTED, (void*) &nAOISupported, sizeof(nAOISupported));
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
	retInt = is_AllocImageMem(hCam, 640, 480, 24, &ppcImgMem, &pid);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
	retInt = is_SetImageMem(hCam, ppcImgMem, pid);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
	//set auto settings
	retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &on, &empty);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
	retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &on, &empty);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
	retInt = is_CaptureVideo(hCam, IS_WAIT);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
}

UeyeOpencvCam::~UeyeOpencvCam() {
	int retInt = is_ExitCamera(hCam);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
}

cv::Mat UeyeOpencvCam::getFrame() {
	VOID * pMem;
	int retInt = is_GetImageMem(hCam, &pMem);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
	memcpy(mattie.ptr(), pMem, (640 * 480 * 3));
	return mattie;
}

void UeyeOpencvCam::getFrame(cv::Mat& mat)
{
	VOID* pMem;
	int retInt = is_GetImageMem(hCam, &pMem);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
	//TODO check if mat has right size and bit-depth
	memcpy(mat.ptr(), pMem, 640 * 480 * 3);
}

HIDS UeyeOpencvCam::getHIDS() {
	return hCam;
}

void UeyeOpencvCam::setAutoWhiteBalance(bool set) {
	double empty;
	double on = set ? 1 : 0;
	int retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &on, &empty);
	if (retInt != IS_SUCCESS) {
		throw UeyeOpenCVException(hCam, retInt);
	}
}
