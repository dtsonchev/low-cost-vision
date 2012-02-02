//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        FGBGSeparation_test
// File:           Main.h
// Description:    Program that traines with images.Normal images are used and FGBG images where white means forground and black means background. After training the program can generate the FGBG images self.
// Author:         Franc Pape & Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of FGBGSeparation_test.
//
// FGBGSeparation_test is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// FGBGSeparation_test is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with FGBGSeparation_test.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <cstdlib>
#include <cassert>
#include <string>
#include <sstream>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/timer.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <FGBGSeparation/FGBGSeparation.h>
#include <FGBGSeparation/TrainingData.h>
#include <imageMetaData/Tools.hpp>
#include <report/Report.hpp>
#include <report/ReportList.hpp>
#include <report/ReportHistogram.hpp>
#include <report/CategoryOverview.hpp>

#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

using namespace cv;
using namespace std;
using namespace imageMetaData;
using namespace report;

//TODO RGB of HSV aangeven met de nummers 1 en 2 is niet zo'n goed idee
//TODO als er fouten in een van de XML bestanden staat crasht t programma

// Function that counts the number of pixels that are the same in two images
int compare(Mat& img1, Mat& img2);

int main(int argc, char* argv[]) {
//========================================================================
// Extract and check the input parameters
//========================================================================
	if (argc < 7) {
		cout
				<< "Usage: test <result directory> <training (ton || toff)> <bins> "
				<< "<masksize> <use RGB or HSV colorspace (RGB || HSV)> <xml with testing data> "
				<< "[xml with training data]" << endl;
		return -1;
	}

	string resultDir;
	string training;
	int bins;
	int maskSize;
	string RGBorHSV;
	string testXmlPath;
	string trainXmlPath;
	stringstream ss;

	// Parse the input parameters
	ss << argv[1];
	ss >> resultDir;
	ss.clear();

	ss << argv[2];
	ss >> training;
	ss.clear();
	assert(training == "ton" || training == "toff");

	if (training == "ton" && argc < 8) {
		cout
				<< "Usage: test <result directory> <training (ton || toff)> <bins> "
				<< "<masksize> <use RGB or HSV colorspace (RGB || HSV)> <xml with testing data> "
				<< "[xml with training data]" << endl;
		return -1;
	}

	ss << argv[3];
	ss >> bins;
	ss.clear();
	assert(bins > 1 && bins < 255);

	ss << argv[4];
	ss >> maskSize;
	ss.clear();
	assert(maskSize > 1 && maskSize < 255 && (maskSize % 2 == 1));

	ss << argv[5];
	ss >> RGBorHSV;
	ss.clear();
	assert(RGBorHSV == "RGB" || RGBorHSV == "HSV");

	ss << argv[6];
	ss >> testXmlPath;
	ss.clear();

	if (training == "ton") {
		ss << argv[7];
		ss >> trainXmlPath;
		ss.clear();
	}

//========================================================================
// Create some helper objects
//========================================================================
	// Create the object that will do the work
	FGBGSeparator tree(bins, maskSize, RGBorHSV == "RGB" ? RGB : HSV);
	// Create a stopwatch for timing how long things take
	boost::timer stopwatch;
	// Create a container for storing the result per image
	map<string, double> imgResults;
	// Create a container for storing the categories and their results
	CategoriesResults catsResults;
	// Create a container for the timing results
	ReportList* timeResults = new ReportList("Timing information", 2, STRING,
			DOUBLE);

//========================================================================
// Training phase (Optional)
//========================================================================
	if (training == "ton") {
		// Start the stopwatch for the training phase
		stopwatch.restart();

		// Extract the images from XML
		vector<ImageMD> trainingImages = getMetaData(trainXmlPath,
				"trainingsset");
		foreach(ImageMD img, trainingImages){
		Mat srcImage = imread(img.path);
		Mat binImage = imread(img.properties.at("FB"));
		tree.addImageToTrainingsSet(srcImage, binImage);
	}

	//train the algorithm
		tree.train();
		tree.saveTraining("data/tree.xml", "tree");

		// Store the time taken for the training phase
		timeResults->appendRow("Training", stopwatch.elapsed());
	}

//========================================================================
// Testing phase
//========================================================================
	// Start the stopwatch for the testing phase
	stopwatch.restart();

	if (training == "toff") {
		tree.loadTraining("data/tree.xml", "tree");
	}

	// Load the metadata of the images from an XML file
	vector<ImageMD> images = getMetaData(testXmlPath, "testset");

	// Loop through all the testing images
	foreach(ImageMD& img, images){
	Mat image = imread(img.path);
	Mat result = image.clone();
	tree.separateFB(image, result);

	int equalPixels = compare(image, result);
	double percCorrect = equalPixels / (double)(image.rows * image.cols) * 100;
	imgResults[img.path]= percCorrect;

	// Store the results in the categories container
	foreach(Category c, img.categories) {
		if(percCorrect > 75) {
			// Increment number of correct images in category
			catsResults[c.first][c.second].first++;
		}
		// Increment total number images in category
		catsResults[c.first][c.second].second++;
	}
}

// Store the time taken for the training phase
	timeResults->appendRow("Testing", stopwatch.elapsed());

//========================================================================
// Create the report
//========================================================================
	Report r("Foreground / background separation");
	r.setDescription(
			"This algorithm attempts to separate the foreground and background of an image.");

	string temp;
	// Put the parameters used to run the algorithm in the report
	ReportList* params = new ReportList("Parameters", 2, STRING, STRING);
	ss.str("");
	ss << bins;
	params->appendRow("Bins", ss.str().c_str());
	ss.str("");
	ss << maskSize;
	params->appendRow("Mask size", ss.str().c_str());
	params->appendRow("RGBorHSV", RGBorHSV.c_str());
	r.addField(params);

	// Add the time results to the report
	timeResults->setColumnNames("What", "Time taken (s)");
	r.addField(timeResults);

	// Put the result of each image in a ReportList
	ReportList* allResults = new ReportList("All results", 2, STRING, DOUBLE);
	typedef pair<string, double> imgResult;
	foreach(imgResult imres, imgResults){
	allResults->appendRow(imres.first, imres.second);
}
	allResults->setColumnNames("Image path", "Pixels correct (%)");
	r.addField(allResults);

	// Create a histogram of the results
	ReportHistogram* his = new ReportHistogram("Histogram",
			getAllValues(imgResults), 10, 100);
	his->setColumnNames("Range", "Images >75% correct");
	r.addField(his);

	// Add all categories and their results to the main report
	foreach(CategoryResults cr, catsResults){
	CategoryOverview* cat = new CategoryOverview(cr);
	cat->setColumnNames("Category", "Correct images", "Percentage correct");
	r.addField(cat);
}

// Save the report as an HTML file
	if (r.saveHTML(resultDir + "/results.html")) {
		cout << "Report saved successfully!" << endl;
	} else {
		cerr << "Saving report failed!" << endl;
	}

	return 0;
}

int compare(Mat& img1, Mat& img2) {
	assert(img1.cols == img2.cols && img1.rows == img2.rows);
	int equalPixels = 0;
	MatConstIterator_<Vec3b> it1 = img1.begin<Vec3b>();
	MatConstIterator_<Vec3b> it2 = img2.begin<Vec3b>();
	const MatConstIterator_<Vec3b> img1End = img1.end<Vec3b>();
	//for every pixel
	for (; it1 != img1End; ++it1, ++it2) {
		Vec3b p1 = *it1;
		Vec3b p2 = *it2;
		if (abs(p1[0] - p2[0]) < 127) {
			++equalPixels;
		}
	}

	return equalPixels;
}
