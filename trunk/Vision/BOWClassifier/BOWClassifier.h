/*
 * BoWClassifier.h
 *
 *  Created on: Oct 12, 2011
 *      Author: Jules Blok
 */

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
