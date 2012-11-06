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
