//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        BOWClassifier
// File:           BOWBayesClassifier.h
// Description:    A Bag of Words KeyPoint classifier utilizing Normal Bayes
// Author:         Jules Blok
// Notes:          None
//
// License:        GNU GPL v3
//
// This file is part of BOWClassifier.
//
// BOWClassifier is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// BOWClassifier is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with BOWClassifier.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#ifndef BOWBAYESCLASSIFIER_H_
#define BOWBAYESCLASSIFIER_H_

#include "BOWClassifier.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/ml/ml.hpp"

/*! \brief A Bag of Words KeyPoint classifier
 *         utilizing Normal Bayes
 *
 *  This class allows you to classify an image
 *  according to labels to which it is trained.
 *  This derived class uses Normal Bayes
 *  for the classification.
 */
class BOWBayesClassifier: public BOWClassifier {
public:
	/*! \brief The constructor, allows different feature classes
	 *
	 *  The constructor defaults to a combination of SURF and Flann.
	 *  You can make your own combination using the parameters.
	 *
	 *  \note Not compatible with the ORB or BRIEF descriptor.
	 */
	BOWBayesClassifier(cv::Ptr<cv::FeatureDetector> detector =
			new cv::SurfFeatureDetector(),
			cv::Ptr<cv::DescriptorExtractor> extractor =
					new cv::SurfDescriptorExtractor(),
			cv::Ptr<cv::DescriptorMatcher> matcher =
					new cv::FlannBasedMatcher());
	virtual ~BOWBayesClassifier();

	virtual bool train(const std::vector<std::string>& paths,
			const cv::Mat& labels);

	virtual bool classify(const cv::Mat& image, float& result);

	virtual bool classify(const std::vector<std::string>& paths,
			cv::Mat& results);
};

#endif /* BOWBAYESCLASSIFIER_H_ */
