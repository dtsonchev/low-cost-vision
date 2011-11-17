#include <string>
#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <boost/filesystem.hpp>
#include <ImageMD/Types.hpp>
#include <ImageMD/ImageMD_Tools.hpp>

using namespace std;
using namespace ImageMetaData;
using namespace boost::filesystem;

int main(int argc, char** argv) {
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

	// Loop through all the images
	for(ImageMD& img : images){
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
		for(Category c : img.categories){
			if(test){
				// Increment number of correct images in category
				catsResults[c.first][c.second].first++;
			}
			// Increment total number images in category
			catsResults[c.first][c.second].second++;
		}
	}

	// print the results
	cout << "Results:" << endl;

	for(pair<string, double> res : imgResults){
		path imgPath(res.first);
		cout << setw(18) << imgPath.filename() << setprecision(2) << res.second << endl;
	}

	cout << endl << endl;

	for(CategoryResults catResults : catsResults){
		cout << catResults.first << ":" << endl;
		for(SubCategoryResults subCatResults : catResults.second){
			cout << " - ";
			cout << setw(15) << subCatResults.first;
			cout << subCatResults.second.first << ":" << subCatResults.second.second << endl;
		}
		cout << endl;
	}

	cout << "The end" << endl;

	return 0;
}
