#ifndef RECTIFYIMAGE_H_
#define RECTIFYIMAGE_H_

#include <opencv2/core/core.hpp>
/**
 * @brief this class creates and uses a matrix in order to rectify an image \n
 * the matrix is created by loading a directory with checker board pattern images
 */
class RectifyImage {
private:
	std::vector<std::vector<cv::Point3f> > objectPoints;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	cv::Mat distCoeffs;
	cv::Mat cameraMatrix;
	cv::Mat map1;
	cv::Mat map2;

	void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
	double calibrate(cv::Size &imageSize);
public:
	/**
	 * Creates a matrix from all the images located in imageDir and stores it in the XMl file
	 *
	 * @param imageDir the directory which contains the image to rectify
	 * @param boardSize amount of squares horizontally -1, amount of squares vertically -1
	 * @param XMLName the name of the xml file were the matrix for rectification is written to
	 * @return the amount of images which are successfully processed
	 */
	int createXML(const char* imageDir, const cv::Size &boardSize, const char* XMLName);
	/**
	 * this function loads a matrix to rectify
	 *
	 * @param XMLName the name of the XML
	 * @param imageSize the size of the image
	 * @return <i>false</i> if XMLName is not available
	 */
	bool initRectify(const char* XMLName, const cv::Size &imageSize);
	/**
	 * Returns the given image rectified
	 *
	 * @param input The image that needs to be rectified
	 * @param output The rectified image
	 */
	void rectify(const cv::Mat &input, cv::Mat &output);
};

#endif /* RECTIFYIMAGE_H_ */
