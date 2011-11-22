/*
 * BoWDTreeClassifier.h
 *
 *  Created on: Oct 18, 2011
 *      Author: Jules Blok
 */

#ifndef BOWDTREECLASSIFIER_H_
#define BOWDTREECLASSIFIER_H_

#include "BOWClassifier.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/ml/ml.hpp"

/*! \brief A Bag of Words KeyPoint classifier
 *         utilizing Decision Trees
 *
 *  This class allows you to classify an image
 *  according to labels to which it is trained.
 *  This derived class uses Decision Trees
 *  for the classification.
 */
class BOWDTreeClassifier : public BOWClassifier {
public:
	/*! \brief The constructor, allows different feature classes
	 *
	 *  The constructor defaults to a combination of SURF and Flann.
	 *  You can make your own combination using the parameters.
	 *
	 *  \note Not compatible with the ORB or BRIEF descriptor.
	 */
	BOWDTreeClassifier(cv::Ptr<cv::FeatureDetector> detector = new cv::SurfFeatureDetector(),
			cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor(),
			cv::Ptr<cv::DescriptorMatcher> matcher = new cv::FlannBasedMatcher());
	virtual ~BOWDTreeClassifier();

	/*! \brief Trains the classifier
	 *
	 *  Trains the classifier on a list of images with custom parameters.
	 *
	 *  \param paths List of training image paths
	 *  \param labels Matrix containing ordered labels for each image
	 *  \param params Training parameters for the neural network
	 *  \return <i>true</i> if successfully trained\n
	 *  		<i>false</i> if reading failed or no keypoints were found
	 */
	bool train(const std::vector<std::string>& paths, const cv::Mat& labels, CvDTreeParams params);

	virtual bool train(const std::vector<std::string>& paths, const cv::Mat& labels);

	virtual bool classify(const cv::Mat& image, float& result);

	virtual bool classify(const std::vector<std::string>& paths, cv::Mat& results);
};

#endif /* BOWDTREECLASSIFIER_H_ */
