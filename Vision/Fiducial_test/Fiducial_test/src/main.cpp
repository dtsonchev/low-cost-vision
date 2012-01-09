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
// License:        GNU GPL v3
//
// This file is part of Fiducial_test.
//
// Fiducial_test is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Fiducial_test is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Fiducial_test.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <string>
#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <boost/foreach.hpp>
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
#define MAX_RANGE 5

using namespace std;
using namespace imageMetaData;
using namespace report;

inline float dist(cv::Point2f p1, cv::Point2f p2) { return(sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))); }

int main(int argc, char** argv){
        if (argc < 3) {
                cout << "Usage: " << argv[0] << " <xml path> <xml root tag>\n";
                return 1;
        }

        cout << setiosflags(ios::left) << setiosflags(ios::fixed);

        // Load the metadata of the images from an XML file
        vector<ImageMD> images = getMetaData(argv[1], argv[2]);

        // Create a container for storing the result per image.
        // Here we use a double as result, but this can be anything
        map<string, double> fidResults;
        map<string, double> qrResults;
        map<string, double> calibResults;

        map<string, int> fidCounts;
        map<string, int> qrCounts;
        map<string, int> calibCounts;

        // Create a container for storing the categories and their results
        CategoriesResults fidCats;
        CategoriesResults qrCats;
        CategoriesResults calibCats;

        // The vision algorithm
        FiducialDetector fidDetector;
        CrateDetector crateDetector;
        QRCodeDetector qrDetector;

        // Loop through all the images
        foreach(ImageMD& img, images) {
			bool fidSuccess = false;
			bool qrSuccess = false;
			bool calibSuccess = false;
			fidCounts[img.path] = 0;
			qrCounts[img.path] = 0;
			calibCounts[img.path] = 0;

			cv::Mat image = cv::imread(img.path, CV_LOAD_IMAGE_GRAYSCALE);
			if(!image.data) {
				cout << "Image " << img.path << " not found! Aborting test..." << endl;
				return 1;
			}
			cout << "Testing image " << img.path << endl;

			vector<cv::Point2f> points;
			vector<cv::Point2f> calib;
			vector<Crate> fidCrates;
			vector<Crate> qrCrates;
			fidDetector.detect(image, points);
			crateDetector.detect(image, points, fidCrates, &calib);
			qrDetector.detectCrates(image, qrCrates);

			for(std::vector<Crate>::iterator it=fidCrates.begin(); it!=fidCrates.end(); ++it) it->draw(image);
			for(std::vector<Crate>::iterator it=qrCrates.begin(); it!=qrCrates.end(); ++it) it->draw(image);
			//cv::imshow("Image", image);
			//cv::waitKey();

			foreach(Properties object, img.objects) {
				if(object["type"] == "QR code") {
					if(!qrCrates.empty()) {
						qrCounts[img.path]++;
						qrSuccess = true;
						cv::Point2f ptTarget((double)object["x"], (double)object["y"]);
						float lastDistance = dist(qrCrates[0].rect().center, ptTarget);
						Crate result = qrCrates[0];

						// Get the crate closest to the target
						foreach(Crate crate, qrCrates) {
							float d = dist(crate.rect().center,ptTarget);
							if(d < lastDistance) {
								result = crate;
								lastDistance = d;
							}
						}

						vector<cv::Point2f> targetPoints;
						targetPoints.push_back(cv::Point2f((double)object["marker1.x"], (double)object["marker1.y"]));
						targetPoints.push_back(cv::Point2f((double)object["marker2.x"], (double)object["marker2.y"]));
						targetPoints.push_back(cv::Point2f((double)object["marker3.x"], (double)object["marker3.y"]));
						float totalDistance = 0;

						for(int i=0; i<3; i++) {
							float d = dist(result.points[i], targetPoints[i]);
							if(d > MAX_DEVIATION) {
								qrSuccess = false;
							}
							totalDistance += d;
						}

						qrResults[img.path] += totalDistance / 3.0;
					}
					else {
						qrSuccess = false;
						qrResults[img.path] = -1.0;
					}
				}
				else if(object["type"] == "Crate") {
					if(!fidCrates.empty()) {
						fidCounts[img.path]++;
						fidSuccess = true;
						cv::Point2f ptTarget((double)object["x"], (double)object["y"]);
						float lastDistance = dist(fidCrates[0].rect().center, ptTarget);
						Crate result = fidCrates[0];

						// Get the crate closest to the target
						foreach(Crate crate, fidCrates) {
							float d = dist(crate.rect().center,ptTarget);
							if(d < lastDistance) {
								result = crate;
								lastDistance = d;
							}
						}

						vector<cv::Point2f> targetPoints;
						targetPoints.push_back(cv::Point2f((double)object["fid1.x"], (double)object["fid1.y"]));
						targetPoints.push_back(cv::Point2f((double)object["fid2.x"], (double)object["fid2.y"]));
						targetPoints.push_back(cv::Point2f((double)object["fid3.x"], (double)object["fid3.y"]));
						float totalDistance = 0;

						for(int i=0; i<3; i++) {
							float d = dist(result.points[i], targetPoints[i]);
							if(d > MAX_DEVIATION) {
								fidSuccess = false;
							}
							totalDistance += d;
						}

						fidResults[img.path] += totalDistance / 3.0;
					}
					else {
						fidSuccess = false;
						fidResults[img.path] = -1.0;
					}
				}
				else if(object["type"] == "Marker") {
					if(!calib.empty()) {
						calibCounts[img.path]++;
						calibSuccess = true;
						cv::Point2f ptTarget((double)object["x"], (double)object["y"]);
						float lastDistance = dist(calib[0], ptTarget);
						cv::Point2f result = calib[0];

						// Get the crate closest to the target
						foreach(cv::Point2f point, calib) {
							float d = dist(point,ptTarget);
							if(d < lastDistance) {
								result = point;
								lastDistance = d;
							}
						}

						calibResults[img.path] += dist(result, ptTarget);
					}
					else {
						calibSuccess = false;
						calibResults[img.path] = -1.0;
					}
				}
			}

			if(fidCounts[img.path] != 0) fidResults[img.path] /= fidCounts[img.path];
			// Store the results in the categories container
			foreach(Category c, img.categories) {
				if(fidSuccess){
					// Increment number of correct images in category
					fidCats[c.first][c.second].first++;
				}
				// Increment total number images in category
				fidCats[c.first][c.second].second++;
			}

			if(qrCounts[img.path] != 0) qrResults[img.path] /= qrCounts[img.path];
			// Store the results in the categories container
			foreach(Category c, img.categories) {
				if(qrSuccess){
					// Increment number of correct images in category
					qrCats[c.first][c.second].first++;
				}
				// Increment total number images in category
				qrCats[c.first][c.second].second++;
			}

			if(calibCounts[img.path] != 0) calibResults[img.path] /= calibCounts[img.path];
			if(calibCounts[img.path] < 3) calibSuccess = false;
			// Store the results in the categories container
			foreach(Category c, img.categories) {
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
        typedef pair<string, double> imgResult;
        typedef pair<string, int> imgCount;

        /* Fiducial */

        // Put the result of each image in a ReportList
        ReportList* fidList = new ReportList("Fiducial results", 2, STRING, DOUBLE);
        foreach(imgResult imres, fidResults) {
        	fidList->appendRow(imres.first, imres.second);
        }
        fidList->setColumnNames("Image path", "Mean deviation");
        r.addField(fidList);

        // Put the count of each image in a ReportList
        ReportList* fidCount = new ReportList("Fiducial counts", 2, STRING, INT);
        foreach(imgCount imres, fidCounts) {
        	fidCount->appendRow(imres.first, imres.second);
        }
        fidCount->setColumnNames("Image path", "Crate count");
        r.addField(fidCount);


        // Create a histogram of the results
        ReportHistogram* fidHis = new ReportHistogram("Fiducial histogram", getAllValues(fidResults), MAX_RANGE*2, MAX_RANGE);
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
        ReportList* qrList = new ReportList("QRCode results", 2, STRING, DOUBLE);
        foreach(imgResult imres, qrResults) {
        	qrList->appendRow(imres.first, imres.second);
        }
        qrList->setColumnNames("Image path", "Mean deviation");
        r.addField(qrList);

        // Put the count of each image in a ReportList
        ReportList* qrCount = new ReportList("QRCode counts", 2, STRING, INT);
        foreach(imgCount imres, qrCounts) {
        	qrCount->appendRow(imres.first, imres.second);
        }
        qrCount->setColumnNames("Image path", "Crate count");
        r.addField(qrCount);

        // Create a histogram of the results
        ReportHistogram* qrHis = new ReportHistogram("QRCode histogram", getAllValues(qrResults), MAX_RANGE*2, MAX_RANGE);
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
        ReportList* calibList = new ReportList("Calibration results", 2, STRING, DOUBLE);
        foreach(imgResult imres, calibResults) {
        	calibList->appendRow(imres.first, imres.second);
        }
        calibList->setColumnNames("Image path", "Mean deviation");
        r.addField(calibList);

        // Put the count of each image in a ReportList
        ReportList* calibCount = new ReportList("Calibration counts", 2, STRING, INT);
        foreach(imgCount imres, calibCounts) {
        	calibCount->appendRow(imres.first, imres.second);
        }
        calibCount->setColumnNames("Image path", "Marker count");
        r.addField(calibCount);

        // Create a histogram of the results
        ReportHistogram* calibHis = new ReportHistogram("Calibration histogram", getAllValues(calibResults), MAX_RANGE*2, MAX_RANGE);
        calibHis->setColumnNames("Deviation range", "Images with deviation");
        r.addField(calibHis);

        // Add all categories and their results to the main report
        foreach(CategoryResults cr, calibCats) {
                CategoryOverview* cat = new CategoryOverview(cr);
                cat->setColumnNames("Category", "Correct images", "Percentage correct");
                r.addField(cat);
        }

        r.saveHTML("results.html");

        cout << "Fiducial testbench done" << endl;

        return 0;
}

