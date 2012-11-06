//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        BOWClassifier
// File:           BOWBayesClassifier.cpp
// Description:    A Bag of Words KeyPoint classifier utilizing Normal Bayes
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

#include "BOWClassifier/BOWBayesClassifier.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "boost/filesystem.hpp"
#include <vector>
#include <fstream>
#include <iostream>

using namespace cv;
using namespace std;
using namespace boost::filesystem;

BOWBayesClassifier::BOWBayesClassifier(cv::Ptr<cv::FeatureDetector> detector,
		cv::Ptr<cv::DescriptorExtractor> extractor,
		cv::Ptr<cv::DescriptorMatcher> matcher) {
	this->detector = detector;
	this->extractor = extractor;
	this->matcher = matcher;
	this->bowExtractor = new BOWImgDescriptorExtractor(extractor, matcher);
	this->classifier = new CvNormalBayesClassifier();
}

BOWBayesClassifier::~BOWBayesClassifier() {
}

bool BOWBayesClassifier::train(const std::vector<std::string>& paths,
		const cv::Mat& labels) {
	// Construct the BoW trainer
	BOWKMeansTrainer trainer(paths.size(), // Dictionary size
			TermCriteria(CV_TERMCRIT_ITER, 10, 0.001), // Criteria
			1, // Retries
			KMEANS_PP_CENTERS // Heuristic KMeans
			);

	cout << "Training vocabulary..." << endl;

	// Train vocabulary
	for (vector<string>::const_iterator iter = paths.begin();
			iter != paths.end(); iter++) {
		cout << "Processing file " << *iter << endl;
		Mat img = imread(*iter); // Read the file

		// Check for invalid input
		if (!img.data) {
			cout << "Could not open " << *iter << endl;
			return false;
		}

		// vector of keypoints
		vector<KeyPoint> keypoints;
		detector->detect(img, keypoints);

		if (keypoints.empty()) {
			cout << "No keypoints detected in " << *iter << endl;
			return false;
		}

		// Extraction of the SURF descriptors
		Mat descriptors;
		extractor->compute(img, keypoints, descriptors);

		// Add features to trainer
		trainer.add(descriptors);
	}

	cout << "Clustering features..." << endl;

	// Generate dictionary
	Mat dictionary = trainer.cluster();
	bowExtractor->setVocabulary(dictionary);

	// Train classifier
	Mat trainingData(0, paths.size(), CV_32FC1);
	for (vector<string>::const_iterator iter = paths.begin();
			iter != paths.end(); iter++) {
		cout << "Processing file " << *iter << endl;
		Mat img = imread(*iter); // Read the file

		// Check for invalid input
		if (!img.data) {
			cout << "Could not open " << *iter << endl;
			return false;
		}

		// vector of keypoints
		vector<KeyPoint> keypoints;
		// Detect the SURF features
		detector->detect(img, keypoints);

		if (keypoints.empty()) {
			cout << "No keypoints detected in " << *iter << endl;
			return false;
		}

		// Extract BoW descriptors
		Mat bowDescriptor;
		bowExtractor->compute(img, keypoints, bowDescriptor);

		// Add and label training data
		trainingData.push_back(bowDescriptor);
	}

	cout << "Training classifier..." << endl;
	return static_cast<CvNormalBayesClassifier*>(classifier)->train(
			trainingData, labels);
}

bool BOWBayesClassifier::classify(const cv::Mat& image, float& result) {
	// Vector of keypoints
	vector<KeyPoint> keypoints;
	// Detect the SURF features
	detector->detect(image, keypoints);

	// If no keypoints were found, return error
	if (keypoints.empty())
		return false;

	// Extraction of the BoW descriptors
	Mat descriptors;
	bowExtractor->compute(image, keypoints, descriptors);

	// Run the classifier
	result = static_cast<CvNormalBayesClassifier*>(classifier)->predict(
			descriptors);

	return true;
}

bool BOWBayesClassifier::classify(const std::vector<std::string>& paths,
		cv::Mat& results) {
	Mat descriptors(0, paths.size(), CV_32FC1);
	for (vector<string>::const_iterator iter = paths.begin();
			iter != paths.end(); iter++) {
		cout << "Processing file " << *iter << endl;
		Mat img = imread(*iter); // Read the file

		// Check for invalid input
		if (!img.data) {
			cout << "Could not open " << *iter << endl;
			return false;
		}

		// vector of keypoints
		vector<KeyPoint> keypoints;
		// Detect the SURF features
		detector->detect(img, keypoints);

		if (keypoints.empty()) {
			cout << "No keypoints detected in " << *iter << endl;
			return false;
		}

		// Extract BoW descriptors
		Mat bowDescriptor;
		bowExtractor->compute(img, keypoints, bowDescriptor);

		// Add and label training data
		descriptors.push_back(bowDescriptor);
	}

	// Run the classifier
	static_cast<CvNormalBayesClassifier*>(classifier)->predict(descriptors,
			&results);

	return true;
}
