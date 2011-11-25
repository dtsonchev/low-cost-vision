/*
 * FiducialDetector.h
 *
 *  Created on: Nov 2, 2011
 *      Author: Jules Blok
 */

#ifndef FIDUCIALDETECTOR_H_
#define FIDUCIALDETECTOR_H_

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>

/*! \brief Detects fiducial markers.
 *
 *  This class allows you to locate the cross.png
 *  and the crosshair.png fiducials in an image.
 */
class FiducialDetector {
private:
	void polarLine(cv::Mat& image, float rho, float theta, cv::Scalar color,
			int thickness);
	bool detectMedianCenterLine(cv::Vec2f& centerLine, std::vector<cv::Vec2f> lines, cv::Mat* debugImage=NULL);
	bool detectMeanCenterLine(cv::Vec2f& centerLine, std::vector<cv::Vec2f> lines, cv::Mat* debugImage=NULL);
public:
	//! Turn on console debug messages
	bool verbose;
	//! Minimum circle radius
	int minRad;
	//! Maximum circle radius
	int maxRad;
	//! Minimum distance between circles
	int distance;
	//! Vote threshold for circles
	int circleVotes;
	//! Minimum vote threshold for lines, thinner lines get less votes.
	int lineVotes;
	//! Maximum amount of lines that need to be found.
	unsigned int maxLines;
	//! Minimum distance between lines to use for the center line.
	float minDist;
	//! Maximum distance between lines to use for the center line.
	float maxDist;
	//! First canny threshold
	int lowThreshold;
	//! Second canny threshold
	int highThreshold;

	/*! \brief The FiducialDetector constructor
	 *
	 *  Constructs the fiducial detector with default thresholds.
	 *  Supply more arguments to change the thresholds or change
	 *  the public fields after construction.
	 */
	FiducialDetector(int minRad = 20, int maxRad = 40, int distance = 70,
			int circleVotes = 100, float minDist = 1.5f, float maxDist = 5.0f,
			int lineVotes = 10, unsigned int maxLines = 10, int lowThreshold = 125,
			int highThreshold = 300);
	virtual ~FiducialDetector();

	/*! \brief Detects all fiducials in an image
	 *
	 *  Detects all fiducials in the image and automatically
	 *  calls detectCrosshair for each fiducial adding the
	 *  center points to the points vector.
	 *
	 *  \param image Image with the fiducials
	 *  \param points Output vector that will contain the
	 *  center points
	 *  \param debugImage Output image where debug information
	 *  will be drawn on, set to NULL for no debug information
	 */
	void detect(cv::Mat& image, std::vector<cv::Point2f>& points,
			cv::Mat* debugImage = NULL);

	/*! \brief Detects the center point
	 *
	 *  Automatically called by detect(). After the fiducial has
	 *  been segmented this function will determine the center
	 *  point of the crosshair.
	 *
	 *  \sa detect()
	 *
	 *  \param image Image with the crosshair
	 *  \param center Output point that will be set to the
	 *  center point
	 *  \param mask Operation mask of the same size as image
	 *  \param debugImage Output image where debug information
	 *  will be drawn on, set to NULL for no debug information
	 *  \return <i>true</i> if center point was detected\n
	 *  <i>false</i> if detection failed
	 */
	bool detectCrosshair(cv::Mat& image, cv::Point2f& center,
			const cv::Mat& mask = cv::Mat(), cv::Mat* debugImage = NULL);
};

#endif /* FIDUCIALDETECTOR_H_ */
