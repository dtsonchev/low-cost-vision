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


#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

using namespace std;
using namespace imageMetaData;
using namespace report;

int main(int argc, char** argv){
	if (argc < 3) {
		cout << "Usage: main <xml path> <xml root tag>\n";
		return 1;
	}

	cout << setiosflags(ios::left) << setiosflags(ios::fixed);

	// Load the metadata of the images from an XML file
	vector<ImageMD> images = getMetaData(argv[1], argv[2]);

	// Create a container for storing the categories and their results
	CategoriesResults catsResults;

	map<string, double> imgResults;
	vector<double> histResults;

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
			histResults.push_back(deviation);
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

//========================================================================
// Create the report
//========================================================================
	Report r;

	foreach(CategoryResults cr, catsResults){
		CategoryOverview* cat = new CategoryOverview(cr);
		cat->setColumnNames("Category", "Correct images", "Percentage correct");
		r.addField(cat);
		cout << cat->toString() << endl;
	}

	ReportHistogram his("Histogram", histResults, 10, 1);
	his.setColumnNames("Range", "Correct images");
	r.addField(&his);
	cout << his.toString();

	r.saveHTML("results.html");

	return 0;
}
