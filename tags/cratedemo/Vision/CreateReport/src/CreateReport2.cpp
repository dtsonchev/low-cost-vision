//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CreateReport
// File:           CreateReport2.cpp
// Description:    Example for using the CreateReport framework.
// Author:         Wouter Langerak
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of CreateReport.
//
// CreateReport is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// CreateReport is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with CreateReport.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <Cell.hpp>
#include <subCells.hpp>
#include <ReportList.hpp>
#include <ImageMD/Types.hpp>
#include <ImageMD/ImageMD_Tools.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <map>
#include <vector>
#include <CategoryOverview.hpp>
#include <ReportHistogram.hpp>

#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

using namespace std;
using namespace ImageMetaData;

int main(int argc, char** argv) {

	using std::vector;
	vector<Type> col;
	col.push_back(STRING);
	col.push_back(INT);
	col.push_back(DOUBLE);
	ReportList reportList(col);
	const char * one = "one";
	int two = 2;
	double three = 3.3;
	reportList.appendRow(one,two,three);
	reportList.appendRow(one,two,three);
	reportList.appendRow(one,two,three);
	reportList.appendRow(one,two,three);
	reportList.appendRow(one,two,three);
	reportList.appendRow(one,two,three);
	reportList.appendRow(one,two,three);
	reportList.appendRow(one,two,three);
	reportList.appendRow(one,two,three);

	reportList.enableAverageRow(true);
	reportList.enableSumRow(true);
	cout << reportList.toString();

	cout << endl;
	cout << endl;
	vector<Type> col2;

	col2.push_back(DOUBLE);
	col2.push_back(INT);
	ReportList reportList2(col2);
	reportList2.appendRow(2.02,10);
	reportList2.appendRow(2.02,10);
	reportList2.appendRow(2.02,10);
	reportList2.appendRow(2.02,10);

	reportList2.enablePercentRow(true,0,1);
	cout << reportList2.toString();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Load the metadata of the images from an XML file
		vector<ImageMD> images = ImageMetaData::getMetaData("/home/wouter/workspace/CreateReport/misc/test.xml", "Test_set");

		// Create a container for storing the categories and their results
		CategoriesResults catsResults;

		map<string, double> imgResults;

		// Loop through all the images
		foreach(ImageMD& img, images){
			bool test = false;
			// Do something with the image
			// ...

			int width = 1200; // int width = image.width;
			// Check if the obtained result corresponds with the given property
			if(!img.objects.empty() && ContainsKey(img.objects[0], string("x"))){
				double deviation = abs(12 - img.objects[0]["x"]) / (double)width;
				imgResults[img.path] = deviation;
				test = (deviation <= 0.1);
			}

			// Store the results in the categories container
			foreach(Category c, img.categories){
				if(test){
					// Increment number of correct images in category
					catsResults[c.first][c.second].first++;
				}
				// Increment total number images in category
				catsResults[c.first][c.second].second++;
			}
		}

		CatergoryOverview cat = CatergoryOverview(*catsResults.begin());
		cout << cat.toString();
		std::vector<double> list;
	list.push_back(8.1);
	list.push_back(13.1);
	list.push_back(18.1);
	list.push_back(22.1);
	list.push_back(34.1);
	list.push_back(34.1);
	list.push_back(15.1);
	list.push_back(80.1);
	list.push_back(90.1);
	list.push_back(99.1);
	list.push_back(3.1);

	ReportHistogram rH(list,100,100);
	cout << rH.toString();

	return 0;
}
