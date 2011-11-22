#include <BOWClassifier/BOWSVMClassifier.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <vector>
#include <fstream>
#include <iostream>

using namespace cv;
using namespace std;
using namespace boost::filesystem;

BOWSVMClassifier::BOWSVMClassifier(cv::Ptr<cv::FeatureDetector> detector,
		cv::Ptr<cv::DescriptorExtractor> extractor,
		cv::Ptr<cv::DescriptorMatcher> matcher) {
	this->detector = detector;
	this->extractor = extractor;
	this->matcher = matcher;
	this->bowExtractor = new BOWImgDescriptorExtractor(extractor, matcher);
	this->classifier = new CvSVM();
}

BOWSVMClassifier::~BOWSVMClassifier() {
}

bool BOWSVMClassifier::train(const std::vector<std::string>& paths,
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
	return static_cast<CvSVM*>(classifier)->train(trainingData, labels);
}

bool BOWSVMClassifier::classify(const cv::Mat& image, float& result) {
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
	result = static_cast<CvSVM*>(classifier)->predict(descriptors);

	return true;
}

bool BOWSVMClassifier::classify(const std::vector<std::string>& paths,
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
	static_cast<CvSVM*>(classifier)->predict(descriptors, &results);

	return true;
}
