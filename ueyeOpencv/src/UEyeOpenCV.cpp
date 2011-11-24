#include <UEyeOpenCV.hpp>
#include <iostream>
#include <ueye.h>

UeyeOpencvCam::UeyeOpencvCam(int id) {
	hCam = id;
	mattie = cv::Mat(480, 640, CV_8UC3);

	using std::cout;
	using std::endl;
	cout << "is_InitCamera\t" << is_InitCamera(&hCam, 0)<<endl;
	cout << "is_SetColorMode\t" << is_SetColorMode(hCam, IS_CM_BGR8_PACKED) << endl;
	char* ppcImgMem;
	int pid;
	INT nAOISupported = 0;
	cout << "is_ImageFormat\t" << is_ImageFormat(hCam, IMGFRMT_CMD_GET_ARBITRARY_AOI_SUPPORTED, (void*) &nAOISupported, sizeof(nAOISupported)) << endl;
	cout << "is_AllocImageMem\t" << is_AllocImageMem(hCam, 640, 480, 24, &ppcImgMem, &pid) << endl;
	cout << "is_SetImageMem\t" << is_SetImageMem(hCam, ppcImgMem, pid) << endl;
	cout << "is_SetBinning(hCam, IS_BINNING_DISABLE)" << is_SetBinning(hCam, IS_BINNING_DISABLE) << endl;
	cout << "is_SetExternalTrigger(hCam, IS_SET_TRIG_OFF)" << is_SetExternalTrigger(hCam, IS_SET_TRIGGER_OFF) << endl;
	double on = 1;
	double empty;
	//set auto settings
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &on, &empty);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &on, &empty);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &on, &empty);
	cout << "is_CaptureVideo\t" << is_CaptureVideo(hCam, IS_WAIT) << endl;
}

void UeyeOpencvCam::stopCam() {
	is_ExitCamera(hCam);
}


cv::Mat UeyeOpencvCam::getFrame() {
	VOID * pMem;
	is_GetImageMem(hCam, &pMem);
	memcpy(mattie.ptr(), pMem, (640 * 480 * 3));
	return mattie;
}




