//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        Fiducial_test
// File:           main.cpp
// Description:    Tests the fiducial project
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

#include <string>
#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <imageMetaData/Types.hpp>
#include <imageMetaData/Tools.hpp>
#include <report/Report.hpp>
#include <report/CategoryOverview.hpp>
#include <report/ReportHistogram.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "FiducialDetector.h"
#include "CrateDetector.h"
#include "QRCodeDetector.h"
#include "Crate.h"


#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

#define MAX_DEVIATION 5
#define MAX_DISTANCE 50
#define MAX_RANGE 5

using namespace std;
using namespace imageMetaData;
using namespace report;

// The vision algorithms
FiducialDetector fidDetector;
CrateDetector crateDetector;
QRCodeDetector qrDetector;

void config(bool debugMode) {
    fidDetector.verbose = debugMode;
}

int main(int argc, char** argv){
        if (argc < 2) {
                cout << "Usage: " << argv[0] << " <xml path> [debug]\n";
                return 1;
        }

        cout << setiosflags(ios::left) << setiosflags(ios::fixed);

        // Check if debug mode is enabled
        bool debugMode = false;
        if(argc > 2) debugMode = strcmp(argv[2], "debug") == 0;
        config(debugMode);

        // Load the metadata of the images from an XML file
        vector<ImageMD> images = getMetaData(argv[1]);

        // Create containers for storing the result per image.
        unsigned int size = images.size();

        // Deviation results
        vector<double> crateResults(size);
        vector<double> qrResults(size);
        vector<double> calibResults(size);

        // Expected results
        vector<int> crateCounts(size);
        vector<int> qrCounts(size);
        vector<int> calibCounts(size);

        // Detected results
        vector<int> crateSizes(size);
        vector<int> qrSizes(size);
        vector<int> calibSizes(size);

        // Matched results
        vector<int> crateMatches(size);
        vector<int> qrMatches(size);
        vector<int> calibMatches(size);

        // Create a container for storing the categories and their results
        CategoriesResults fidCats;
        CategoriesResults qrCats;
        CategoriesResults calibCats;

        // Loop through all the images
        for(unsigned int i=0; i<images.size(); i++) {
			bool fidSuccess = true;
			bool qrSuccess = true;
			bool calibSuccess = true;
			crateCounts[i] = 0;
			qrCounts[i] = 0;
			calibCounts[i] = 0;

			vector<double> fidDeviation;
			vector<double> qrDeviation;
			vector<double> calibDeviation;

			cv::Mat image = cv::imread(images[i].path, CV_LOAD_IMAGE_GRAYSCALE);
			if(!image.data) {
				cout << "Image " << images[i].path << " not found! Aborting test..." << endl;
				return 1;
			}
			cout << "Testing image " << images[i].name << endl;

			vector<cv::Point2f> points;
			vector<Crate> fidCrates;
			vector<Crate> qrCrates;
			vector<cv::Point2f> calib;

			if(debugMode) {
				cv::Mat debug = cv::imread(images[i].path);
				fidDetector.detect(image, points, &debug);
				crateDetector.detect(image, points, fidCrates, &calib);
				qrDetector.detectCrates(image, qrCrates);

				for(std::vector<Crate>::iterator it=qrCrates.begin(); it!=qrCrates.end(); ++it) {
					vector<cv::Point2f> points = it->getPoints();

					// Draw the fiducial points
					cv::circle(debug, points[0], 1, cv::Scalar(255, 0, 0), 2);
					cv::circle(debug, points[1], 1, cv::Scalar(0, 255, 0), 2);
					cv::circle(debug, points[2], 1, cv::Scalar(0, 0, 255), 2);
				}

				cv::imshow(images[i].name, debug);
			} else {
				fidDetector.detect(image, points);
				crateDetector.detect(image, points, fidCrates, &calib);
				qrDetector.detectCrates(image, qrCrates);
			}

			crateSizes[i] = fidCrates.size();
			qrSizes[i] = qrCrates.size();
			calibSizes[i] = calib.size();

			foreach(Properties object, images[i].objects) {
				if(object["type"] == "QR code") {
					qrCounts[i]++;
					if(!qrCrates.empty()) {
						cv::Point2f ptTarget((double)object["x"], (double)object["y"]);
						float lastDistance = Crate::distance(qrCrates[0].rect().center, ptTarget);
						Crate result = qrCrates[0];
						bool found = lastDistance < MAX_DISTANCE;

						// Get the crate closest to the target
						for(unsigned int j=1; j<qrCrates.size(); j++) {
							float d = Crate::distance(qrCrates[j].rect().center,ptTarget);
							if(d < lastDistance && d < MAX_DISTANCE) {
								result = qrCrates[j];
								lastDistance = d;
								found = true;
							}
						}

						if(found) {
							qrMatches[i]++;
							std::vector<cv::Point2f> points = result.getPoints();
							vector<cv::Point2f> targetPoints;
							targetPoints.push_back(cv::Point2f((double)object["marker1.x"], (double)object["marker1.y"]));
							targetPoints.push_back(cv::Point2f((double)object["marker2.x"], (double)object["marker2.y"]));
							targetPoints.push_back(cv::Point2f((double)object["marker3.x"], (double)object["marker3.y"]));

							for(unsigned int j=0; j<3; j++) {
								float d = Crate::distance(points[j], targetPoints[j]);
								qrDeviation.push_back(d);
							}
						}
					}
				}
				else if(object["type"] == "Crate") {
					crateCounts[i]++;
					if(!fidCrates.empty()) {
						cv::Point2f ptTarget((double)object["x"], (double)object["y"]);
						float lastDistance = Crate::distance(fidCrates[0].rect().center, ptTarget);
						Crate result = fidCrates[0];
						bool found = lastDistance < MAX_DISTANCE;

						// Get the crate closest to the target
						for(unsigned int j=1; j<fidCrates.size(); j++) {
							float d = Crate::distance(fidCrates[j].rect().center,ptTarget);
							if(d < lastDistance && d < MAX_DISTANCE) {
								result = fidCrates[j];
								lastDistance = d;
								found = true;
							}
						}

						if(found) {
							crateMatches[i]++;
							std::vector<cv::Point2f> points = result.getPoints();
							vector<cv::Point2f> targetPoints;
							targetPoints.push_back(cv::Point2f((double)object["fid1.x"], (double)object["fid1.y"]));
							targetPoints.push_back(cv::Point2f((double)object["fid2.x"], (double)object["fid2.y"]));
							targetPoints.push_back(cv::Point2f((double)object["fid3.x"], (double)object["fid3.y"]));

							for(unsigned int j=0; j<3; j++) {
								float d = Crate::distance(points[j], targetPoints[j]);
								fidDeviation.push_back(d);
							}
						}
					}
				}
				else if(object["type"] == "Marker") {
					calibCounts[i]++;
					if(!calib.empty()) {
						cv::Point2f ptTarget((double)object["x"], (double)object["y"]);
						float lastDistance = Crate::distance(calib[0], ptTarget);
						cv::Point2f result = calib[0];
						bool found = lastDistance < MAX_DISTANCE;

						// Get the crate closest to the target
						for(unsigned int j=1; j<calib.size(); j++) {
							float d = Crate::distance(calib[j],ptTarget);
							if(d < lastDistance && d < MAX_DISTANCE) {
								result = calib[j];
								lastDistance = d;
								found = true;
							}
						}

						if(found) {
							calibMatches[i]++;
							float d = Crate::distance(result, ptTarget);
							calibDeviation.push_back(d);
						}
					}
				}
			}

			// Determine result
			if(fidDeviation.empty()) {
				crateResults[i] = -1.0;
				fidSuccess = false;
			}
			else {
				crateResults[i] = *(std::max_element(fidDeviation.begin(), fidDeviation.end()));
				if (crateResults[i] > MAX_DEVIATION ||
						crateMatches[i] != crateCounts[i]	||
						crateSizes[i] != crateCounts[i])
				fidSuccess = false;
			}

			// Store the results in the categories container
			foreach(Category c, images[i].categories) {
				if(fidSuccess){
					// Increment number of correct images in category
					fidCats[c.first][c.second].first++;
				}
				// Increment total number images in category
				fidCats[c.first][c.second].second++;
			}

			// Determine result
			if(qrDeviation.empty()) {
				qrResults[i] = -1.0;
				qrSuccess = false;
			}
			else {
				qrResults[i] = *(std::max_element(qrDeviation.begin(), qrDeviation.end()));
				if (qrResults[i] > MAX_DEVIATION ||
						qrMatches[i] != qrCounts[i] ||
						qrSizes[i] != qrCounts[i])
				qrSuccess = false;
			}

			// Store the results in the categories container
			foreach(Category c, images[i].categories) {
				if(qrSuccess){
					// Increment number of correct images in category
					qrCats[c.first][c.second].first++;
				}
				// Increment total number images in category
				qrCats[c.first][c.second].second++;
			}

			// Determine result
			if(calibDeviation.empty()) {
				calibResults[i] = -1.0;
				calibSuccess = false;
			}
			else {
				calibResults[i] = *(std::max_element(calibDeviation.begin(), calibDeviation.end()));
				if (calibResults[i] > MAX_DEVIATION ||
						calibMatches[i] != calibCounts[i] ||
						calibSizes[i] != calibCounts[i])
				calibSuccess = false;
			}

			// Store the results in the categories container
			foreach(Category c, images[i].categories) {
				if(calibSuccess){
					// Increment number of correct images in category
					calibCats[c.first][c.second].first++;
				}
				// Increment total number images in category
				calibCats[c.first][c.second].second++;
			}
        }

		//========================================================================
		// Create the report
		//========================================================================
        cout << "Writing report..." << endl;
        Report r("Fiducial report");

        /* Fiducial */

        // Put the result of each image in a ReportList
        ReportList* fidList = new ReportList("CrateDetector results", 5, STRING, INT, INT, INT, DOUBLE);
        for(unsigned int i=0; i<images.size(); i++) {
        	fidList->appendRow(images[i].name, crateSizes[i], crateCounts[i], crateMatches[i], crateResults[i]);
        }
        fidList->setColumnNames("Image", "Detected", "Tagged", "Matched", "Max deviation");
        r.addField(fidList);

        // Create a histogram of the results
        ReportHistogram* fidHis = new ReportHistogram("histogram", crateResults, MAX_RANGE*2, MAX_RANGE);
        fidHis->setColumnNames("Deviation range", "Images with deviation");
        r.addField(fidHis);

        // Add all categories and their results to the main report
        foreach(CategoryResults cr, fidCats) {
                CategoryOverview* cat = new CategoryOverview(cr);
                cat->setColumnNames("Category", "Correct images", "Percentage correct");
                r.addField(cat);
        }

        /* QR Code */

        // Put the result of each image in a ReportList
		ReportList* qrList = new ReportList("QRCodeDetector results", 5, STRING, INT, INT, INT, DOUBLE);
		for(unsigned int i=0; i<images.size(); i++) {
			qrList->appendRow(images[i].name, qrSizes[i], qrCounts[i], qrMatches[i], qrResults[i]);
		}
		qrList->setColumnNames("Image", "Detected", "Tagged", "Matched", "Max deviation");
		r.addField(qrList);

        // Create a histogram of the results
        ReportHistogram* qrHis = new ReportHistogram("histogram", qrResults, MAX_RANGE*2, MAX_RANGE);
        qrHis->setColumnNames("Deviation range", "Images with deviation");
        r.addField(qrHis);

        // Add all categories and their results to the main report
        foreach(CategoryResults cr, qrCats) {
                CategoryOverview* cat = new CategoryOverview(cr);
                cat->setColumnNames("Category", "Correct images", "Percentage correct");
                r.addField(cat);
        }

        /* Calibration marker */

        // Put the result of each image in a ReportList
		ReportList* calibList = new ReportList("Calibration results", 5, STRING, INT, INT, INT, DOUBLE);
		for(unsigned int i=0; i<images.size(); i++) {
			calibList->appendRow(images[i].name, calibSizes[i], calibCounts[i], calibMatches[i], calibResults[i]);
		}
		calibList->setColumnNames("Image", "Detected", "Tagged", "Matched", "Max deviation");
		r.addField(calibList);

        // Create a histogram of the results
        ReportHistogram* calibHis = new ReportHistogram("histogram", calibResults, MAX_RANGE*2, MAX_RANGE);
        calibHis->setColumnNames("Deviation range", "Images with deviation");
        r.addField(calibHis);

        // Add all categories and their results to the main report
        foreach(CategoryResults cr, calibCats) {
                CategoryOverview* cat = new CategoryOverview(cr);
                cat->setColumnNames("Category", "Correct images", "Percentage correct");
                r.addField(cat);
        }

        /* FiducialDetector */

        // Put the result of each setting in a ReportList
		ReportList* fidConf = new ReportList("FiducialDetector settings", 2, STRING, DOUBLE);
		fidConf->setColumnNames("Setting", "Value");
		fidConf->appendRow("Verbose", (double)(fidDetector.verbose?1:0));
		fidConf->appendRow("Min. radius", (double)fidDetector.minRad);
		fidConf->appendRow("Max. radius", (double)fidDetector.maxRad);
		fidConf->appendRow("Circle votes", (double)fidDetector.circleVotes);
		fidConf->appendRow("Line votes", (double)fidDetector.lineVotes);
		fidConf->appendRow("Max. lines", (double)fidDetector.maxLines);
		fidConf->appendRow("Min. line distance", (double)fidDetector.minDist);
		fidConf->appendRow("Max. line distance", (double)fidDetector.maxDist);
		fidConf->appendRow("Low threshold", (double)fidDetector.lowThreshold);
		fidConf->appendRow("High threshold", (double)fidDetector.highThreshold);
		fidConf->appendRow("Circle threshold", (double)fidDetector.circleThreshold);
		fidConf->appendRow("Gaussian blur size", (double)fidDetector.blur);
		fidConf->appendRow("Gaussian blur sigma", (double)fidDetector.sigma);
        r.addField(fidConf);

        // Put the result of each setting in a ReportList
		ReportList* crateConf = new ReportList("CrateDetector settings", 2, STRING, DOUBLE);
		crateConf->setColumnNames("Setting", "Value");
		crateConf->appendRow("Low threshold", (double)crateDetector.lowThreshold);
		crateConf->appendRow("High threshold", (double)crateDetector.highThreshold);
        r.addField(crateConf);

        stringstream filename;

        //get the current time from the clock -- one second resolution
		boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();\

		filename << "Fiducial_" << now.date() << "_" << now.time_of_day() << ".html";

        r.saveHTML(filename.str());

        cout << "Fiducial testbench done" << endl;

        if(debugMode) cv::waitKey();

        return 0;
}

