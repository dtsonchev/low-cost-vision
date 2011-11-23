#ifndef RECTIFYSTEREO_H_
#define RECTIFYSTEREO_H_

#include <opencv2/core/core.hpp>

/**
 * @brief This class creates and uses a matrix in order to rectify an image \n
 * the matrix is created by loading multiple sets of images with checker board pattern
 */
class RectifyStereo {
private:
	///The matrixes used for re-mapping the images
	cv::Mat rmap[2][2];

public:
    ///The valid regions within the two correctified images
	cv::Rect validRoi[2];

    /**
     * This function reads filenames from an xml file
     * @param filename the name of the xml file
     * @param list the list in which the filenames are placed
     * @return <b>false</b> if it didn't succeed
     */
	bool readStringList( const std::string& filename, std::vector<std::string>& list );
    /**
     * Creates a xml file for correcting the images
     * @param imagelist a list with all the file names
     * @param xmlName the name of the resulting xml file
     * @param boardSize The size of the checker board (the amount of horizontal squares -1 , the amount of vertical squares -1)
     * @return <b>false</b> if it didn't succeed
     */
	bool StereoCalib(const std::vector<std::string>& imagelist, const std::string& xmlName, cv::Size boardSize);
    /**
     * Reads the matrixes for rectifying the images
     */
	bool initRectifyImage(const std::string& xmlName);
	/**
	 * The function rectifies two images
	 * @param inputL the left input image
	 * @param outputL the left corrected output image
	 * @param inputR the right input image
	 * @param outputR the right corrected output image
	 * @param drawOnImage when true a box around the region of interest is drawn on the output image default value is <b>false</b>
	 */
	void RectifyImage(const cv::Mat inputL, cv::Mat &outputL,
			const cv::Mat inputR, cv::Mat &outputR, bool drawOnImage = false);
};

#endif /* RECTIFYSTEREO_H_ */
