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
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
