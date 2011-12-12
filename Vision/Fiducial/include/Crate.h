/*
 * Crate.h
 *
 *  Created on: Nov 11, 2011
 *      Author: Jules Blok
 */

#ifndef CRATE_H_
#define CRATE_H_

#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <vector>

class Crate {
private:
	//! Bounding rectangle saved after calling rect()
	cv::RotatedRect bounds;
	//! Fiducial points belonging to the crate returned from points()
	std::vector<cv::Point2f> fidPoints;
	//! Tracks if fidPoints is ordered
	bool fidOrdered;
public:
	/*! \brief The Crate constructor
	 *
	 *  Constructs a crate without any fiducial points
	 */
	Crate();

	/*! \brief The Crate constructor
	 *
	 *  Constructs a crate with the given three fiducial points.
	 */
	Crate(const std::vector<cv::Point2f>& points);

	/*! \brief The Crate copy-constructor
	 *
	 *  Constructs a crate with the same values as the
	 *  given crate, fiducial points are deep-copied.
	 */
	Crate(const Crate& crate);

	//! THe Crate deconstructor
	virtual ~Crate();

	/*! \brief Generate a rotated bounding rectangle
	 *
	 *  Generates a RotatedRect that represents the
	 *  crate.
	 */
	cv::RotatedRect rect();

	/*! \brief Return the fiducial points
	 *
	 *  Returns the fiducial points ordered clockwise.
	 *  Will only order fiducial points on first call.
	 */
	std::vector<cv::Point2f> points();

	/*! \brief Draw the rectangle in the image
	 *
	 *  Draws the rectangle in the image including the
	 *  fiducial points, angle and bounding rectangle.
	 */
	void draw(cv::Mat& image);
};

#endif /* CRATE_H_ */
