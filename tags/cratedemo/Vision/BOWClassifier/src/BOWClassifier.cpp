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
