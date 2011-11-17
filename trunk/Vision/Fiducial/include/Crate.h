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
public:
	//! Fiducial points belonging to the crate, should contain no more than 3 points
	std::vector<cv::Point2f> points;

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
	virtual ~Crate();

	/*! \brief Generate a rotated bounding rectangle
	 *
	 *  Generates a RotatedRect that contains the three
	 *  fiducial points and a fourth point that is
	 *  generated from the angle that makes sure the
	 *  correct size is kept during a rotation.
	 */
	cv::RotatedRect rect();

	/*! \brief Draw the rectangle in the image
	 *
	 *  Draws the rectangle in the image including the
	 *  fiducial points, angle and bounding rectangle.
	 */
	void draw(cv::Mat& image);
};

#endif /* CRATE_H_ */
