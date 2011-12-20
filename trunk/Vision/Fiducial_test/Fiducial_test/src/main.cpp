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
#include "Crate.h"


#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

typedef double RESULT_TYPE;
#define MAX_DEVIATION 0.1
#define MAX_RANGE 10

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
        map<string, RESULT_TYPE> imgResults;
        // Create a container for storing the categories and their results
        CategoriesResults catsResults;

        // The vision algorithm
        FiducialDetector fidDetector;
        CrateDetector crateDetector;
		float totalDistance = 0;

        // Loop through all the images
        foreach(ImageMD& img, images) {
                bool success = true;

                cv::Mat image = cv::imread(img.path, CV_LOAD_IMAGE_GRAYSCALE);
                if(!image.data) {
                	cout << "Image " << img.path << " not found! Aborting test..." << endl;
                	return 1;
                }

                vector<cv::Point2f> points;
                vector<Crate> crates;
                fidDetector.detect(image, points);
                crateDetector.detect(image, crates, points);

                // Check if all crates were detected
                if(crates.size() != img.objects.size()) {
                	success = false;
                }
                else {
					foreach(Properties object, img.objects) {
						cv::Point2f ptTarget((int)object["x"], (int)object["y"]);
						float lastDistance = dist(crates[0].rect().center,ptTarget);
						Crate result = crates[0];

						// Get the crate closest to the target
						foreach(Crate crate, crates) {
							float d = dist(crate.rect().center,ptTarget);
							if(d < lastDistance) {
								result = crate;
								lastDistance = d;
							}
						}

						vector<cv::Point2f> points = result.points();
						vector<cv::Point2f> targetPoints;
						targetPoints.push_back(cv::Point2f((int)object["fid1_x"], (int)object["fid1_y"]));
						targetPoints.push_back(cv::Point2f((int)object["fid2_x"], (int)object["fid2_y"]));
						targetPoints.push_back(cv::Point2f((int)object["fid3_x"], (int)object["fid3_y"]));

						for(int i=0; i<3; i++) {
							float d = dist(points[i], targetPoints[i]);
							if(d > MAX_DEVIATION) {
								success = false;
							}
							totalDistance += d;
						}
					}
                }

				imgResults[img.path] = totalDistance / (3 * crates.size());

                // Store the results in the categories container
                foreach(Category c, img.categories) {
                        if(success){
                                // Increment number of correct images in category
                                catsResults[c.first][c.second].first++;
                        }
                        // Increment total number images in category
                        catsResults[c.first][c.second].second++;
                }
        }

		//========================================================================
		// Create the report
		//========================================================================
        Report r("FiducialDetector report");

        // Put the result of each image in a ReportList
        ReportList* allResults = new ReportList("Fiducial results", 2, STRING, DOUBLE);
        typedef pair<string, RESULT_TYPE> imgResult;
        foreach(imgResult imres, imgResults) {
                allResults->appendRow(imres.first, imres.second);
        }
        allResults->setColumnNames("Image path", "Result");
        r.addField(allResults);

        // Create a histogram of the results
        ReportHistogram* his = new ReportHistogram("Histogram", getAllValues(imgResults), 10, 10);
        his->setColumnNames("Range", "Correct images");
        r.addField(his);

        // Add all categories and their results to the main report
        foreach(CategoryResults cr, catsResults) {
                CategoryOverview* cat = new CategoryOverview(cr);
                cat->setColumnNames("Category", "Correct images", "Percentage correct");
                r.addField(cat);
        }

        r.saveHTML("results.html");

        return 0;
}

