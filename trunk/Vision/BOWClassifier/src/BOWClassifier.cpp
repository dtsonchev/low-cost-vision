#include <BOWClassifier/BOWClassifier.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

BOWClassifier::~BOWClassifier() {
	delete classifier;
}

bool BOWClassifier::load(const std::string& path) {
	cout << "Loading classifier..." << endl;
	classifier->load(path.c_str());

	cout << "Loading vocabulary..." << endl;
	FileStorage fs(path, FileStorage::READ);
	Mat vocabulary;
	fs["vocabulary"] >> vocabulary;
	if (vocabulary.empty()) {
		cout << "Failed to load vocabulary" << endl;
		return false;
	}
	bowExtractor->setVocabulary(vocabulary);
	fs.release();

	return true;
}

bool BOWClassifier::save(const std::string& path) {
	cout << "Saving classifier..." << endl;
	classifier->save(path.c_str());

	cout << "Saving vocabulary..." << endl;
	FileStorage fs(path, FileStorage::APPEND);
	if (!fs.isOpened())
		return false;
	fs << "vocabulary" << bowExtractor->getVocabulary();
	fs.release();

	return true;
}
