#include "RectifyImage.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/filesystem.hpp>
#include <sstream>

using namespace cv;
using namespace std;
using namespace boost::filesystem;

void RectifyImage::addPoints(const vector<Point2f>& imageCorners, const vector<Point3f>& objectCorners){
	imagePoints.push_back(imageCorners);
	objectPoints.push_back(objectCorners);
}

double RectifyImage::calibrate(Size &imageSize){
	vector<Mat> rvecs, tvecs;
	return calibrateCamera(objectPoints,
			imagePoints,	// the image points
			imageSize,		// image size
			cameraMatrix,	// output camera matrix
			distCoeffs,		// output distortion matrix
			rvecs, tvecs,	// Rs, Ts
			0);				// set options
}

int RectifyImage::createXML(char* imageDir, Size &boardSize, char* XMLName){
	vector<Point2f> imageCorners;
	vector<Point3f> objectCorners;

	for (int i=0; i<boardSize.height; i++) {
		for (int j=0; j<boardSize.width; j++) {
			objectCorners.push_back(Point3f(i, j, 0.0f));
		}
	}

	if(!is_directory(imageDir)){
		return -1;
	}

	Mat image;
	int successes = 0;
	for (directory_iterator iter = directory_iterator(imageDir); iter != directory_iterator(); iter++) {
		stringstream s;
		s << iter->path();
		image = imread(s.str().c_str(), 0);

		bool found = findChessboardCorners(image, boardSize, imageCorners);
		if(found){
				cornerSubPix(image, imageCorners, Size(4, 4), Size(-1, -1),
				TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1));

				if (imageCorners.size() == (uint)boardSize.area()) {
					addPoints(imageCorners, objectCorners);
					successes++;
				}
		}
	}
	Size imageSize(image.cols, image.rows);
	calibrate(imageSize);
	image.release();

	if(!successes){
		return -1;
	}

    FileStorage fs(XMLName, FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();
	return successes;
}

int RectifyImage::initRectify(char* XMLName, Size imageSize){
	if(!is_regular_file(XMLName)){
		return -1;
	}

	FileStorage fs(XMLName, FileStorage::READ);
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	initUndistortRectifyMap( cameraMatrix, distCoeffs, Mat(), Mat(), imageSize, CV_32FC1, map1, map2);
	return 0;
}

Mat RectifyImage::rectify(const Mat &image){
	Mat undistorted;

	remap(image, undistorted, map1, map2, INTER_LINEAR);
	return undistorted;
}
