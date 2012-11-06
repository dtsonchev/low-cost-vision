//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        BOWClassifier
// File:           BOWClassifier.h
// Description:    A Bag of Words KeyPoint classifier
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

#ifndef BOWCLASSIFIER_H_
#define BOWCLASSIFIER_H_

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/ml/ml.hpp"

/*! \brief A Bag of Words KeyPoint classifier
 *
 *  This class allows you to classify an image
 *  according to labels to which it is trained.
 *
 *  \sa BOWBayesClassifier, BOWDTreeClassifier,
 *      BOWNeuralClassifier, BOWSVMClassifier
 */
class BOWClassifier {
public:
	//! Pointer to the feature detector
	cv::Ptr<cv::FeatureDetector> detector;

	//! Pointer to the descriptor extractor
	cv::Ptr<cv::DescriptorExtractor> extractor;

	//! Pointer to the matcher
	cv::Ptr<cv::DescriptorMatcher> matcher;

	//! Pointer to the BoW descriptor extractor
	cv::Ptr<cv::BOWImgDescriptorExtractor> bowExtractor;

	//! The classifier
	CvStatModel* classifier;

	virtual ~BOWClassifier();

	/*! \brief Loads the classifier from disk
	 *
	 *  Loads the classifier from the disk instead of training it.
	 *
	 *  \warning Use the same feature classes used to generate
	 *  the classifier!
	 *
	 *  \param path Path to a previously saved classifier
	 *  \return <i>true</i> if successfully loaded\n
	 *  		<i>false</i> if loading failed
	 */
	bool load(const std::string& path);

	/*! \brief Saves the classifier to disk
	 *
	 *  \param path Path to save the classifier to
	 *  \return <i>true</i> if successfully saved\n
	 *  		<i>false</i> if saving failed
	 */
	bool save(const std::string& path);

	/*! \brief Trains the classifier
	 *
	 *  Trains the classifier on a list of images.
	 *
	 *  \param paths List of training image paths
	 *  \param labels Matrix containing ordered labels for each image
	 *  \return <i>true</i> if successfully trained\n
	 *  		<i>false</i> if reading failed or no keypoints were found
	 */
	virtual bool train(const std::vector<std::string>& paths,
			const cv::Mat& labels)=0;

	/*! \brief Classify an image
	 *
	 *  Classifies an image according to it's training set.
	 *  It returns the label passed in train().
	 *
	 *  \param image Image to be classified
	 *  \param result Output parameter for the label
	 *  \return <i>true</i> if successful\n
	 *  		<i>false</i> if no keypoints were found
	 */
	virtual bool classify(const cv::Mat& image, float& result)=0;

	/*! \brief Classify multiple images
	 *
	 *  Classifies an multiple images according to it's
	 *  training set. It returns the label passed in train().
	 *
	 *  \param paths List of image paths to be classified
	 *  \param results Output parameter for the labels
	 *  \return <i>true</i> if successful\n
	 *  		<i>false</i> if no keypoints were found
	 */
	virtual bool classify(const std::vector<std::string>& paths,
			cv::Mat& results)=0;
};

#endif /* BOWCLASSIFIER_H_ */
