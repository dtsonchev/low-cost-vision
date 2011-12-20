//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        BOWClassifier
// File:           BOWNeuralClassifier.h
// Description:    A Bag of Words KeyPoint classifier utilizing Neural Networks
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

#ifndef BOWNEURALCLASSIFIER_H_
#define BOWNEURALCLASSIFIER_H_

#include "BOWClassifier.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/ml/ml.hpp"

/*! \brief A Bag of Words KeyPoint classifier
 *         utilizing Neural Networks
 *
 *  This class allows you to classify an image
 *  according to labels to which it is trained.
 *  This derived class uses Neural Networks
 *  for the classification.
 */
class BOWNeuralClassifier: public BOWClassifier {
private:
	/*! \brief Save original labels
	 *  To keep the interface uniform we need to do some conversion
	 *  to make the labels compatible with neural networks.
	 */
	cv::Mat originalLabels;
public:
	/*! \brief The constructor, allows different feature classes
	 *
	 *  The constructor defaults to a combination of SURF and Flann.
	 *  You can make your own combination using the parameters.
	 *
	 *  \note Not compatible with the ORB or BRIEF descriptor.
	 */
	BOWNeuralClassifier(cv::Mat& layers, cv::Ptr<cv::FeatureDetector> detector =
			new cv::SurfFeatureDetector(),
			cv::Ptr<cv::DescriptorExtractor> extractor =
					new cv::SurfDescriptorExtractor(),
			cv::Ptr<cv::DescriptorMatcher> matcher =
					new cv::FlannBasedMatcher());
	virtual ~BOWNeuralClassifier();

	/*! \brief Trains the classifier
	 *
	 *  Trains the classifier on a list of images with custom parameters.
	 *
	 *  \param paths List of training image paths
	 *  \param labels Matrix containing ordered labels for each image
	 *  \param params Training parameters for the neural network
	 *  \param sampleWeights Matrix containing extra weight for certain samples
	 *  \return <i>true</i> if successfully trained\n
	 *  		<i>false</i> if reading failed or no keypoints were found
	 */
	bool train(const std::vector<std::string>& paths, const cv::Mat& labels,
			CvANN_MLP_TrainParams params, const cv::Mat& sampleWeights =
					cv::Mat());

	virtual bool train(const std::vector<std::string>& paths,
			const cv::Mat& labels);

	virtual bool classify(const cv::Mat& image, float& result);

	virtual bool classify(const std::vector<std::string>& paths,
			cv::Mat& results);
};

#endif /* BOWNEURALCLASSIFIER_H_ */
