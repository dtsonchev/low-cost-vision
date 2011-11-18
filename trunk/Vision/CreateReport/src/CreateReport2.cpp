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

#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif


using namespace std;
using namespace ImageMetaData;

int main(int argc, char** argv) {

//	using std::vector;
//	vector<Type> col;
//	col.push_back(STRING);
//	col.push_back(INT);
//	col.push_back(DOUBLE);
//	ReportList reportList(col);
//	const char * one = "one";
//	int two = 2;
//	double three = 3.3;
//	reportList.appendRow(one,two,three);
//	reportList.appendRow(one,two,three);
//	reportList.appendRow(one,two,three);
//	reportList.appendRow(one,two,three);
//	reportList.appendRow(one,two,three);
//	reportList.appendRow(one,two,three);
//	reportList.appendRow(one,two,three);
//	reportList.appendRow(one,two,three);
//	reportList.appendRow(one,two,three);
//
//	reportList.enableAverageRow(true);
//	reportList.enableSumRow(true);
//	cout << reportList.toString();
//
//	cout << endl;
//	cout << endl;
//	vector<Type> col2;
//
//	col2.push_back(DOUBLE);
//	col2.push_back(INT);
//	ReportList reportList2(col2);
//	reportList2.appendRow(2.02,10);
//	reportList2.appendRow(2.02,10);
//	reportList2.appendRow(2.02,10);
//	reportList2.appendRow(2.02,10);
//	reportList2.enablePercentRow(true,0,1);
//	cout << reportList2.toString();
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
	return 0;
}
