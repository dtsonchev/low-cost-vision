#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dirent.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <limits>
#include "FGBGSeparation.h"

using namespace cv;
using namespace std;

int main(int argc, char* argv[]){

	if(argc < 2 || (strcmp(argv[1], "-t") && strcmp(argv[1], "-d") && strcmp(argv[1], "-td"))){
		cout << "Wether to train (-t), decided (-d) or both (-td)" << endl;
		return 0;
	}else if(argc < 6){
		if(!strcmp(argv[1], "-t")){
			cout << "Wether to train (-t) or decided (-d),"
					"\nNeed amount of bins,"
					"\nMask size for histograms,"
					"\nRGB '1' or HSV '2'" << endl;
			return 0;
		}else if(!strcmp(argv[1], "-d")){
			cout << "Wether to train (-t) or decided (-d),"
					"\nNeed amount of bins,"
					"\nMask size for histograms,"
					"\nRGB '1' or HSV '2',"
					"\nImage to decide upon" << endl;
			return 0;
		}else if(!strcmp(argv[1], "-td")){
			cout << "Wether to train (-t) or decided (-d),"
					"\nNeed amount of bins,"
					"\nMask size for histograms,"
					"\nRGB '1' or HSV '2',"
					"\nImage to decide upon," << endl;
			return 0;
		}
	}else if(argc < 6 && !strcmp(argv[1], "-td")){
		cout << "Wether to train (-t) or decided (-d),"
				"\nNeed amount of bins,"
				"\nMask size for histograms,"
				"\nRGB '1' or HSV '2',"
				"\nImage to decide upon," << endl;
		return 0;
	}
	try{
		FGBGSeparator tree(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));

		if(!strcmp(argv[1], "-t") || !strcmp(argv[1], "-td")){
			cout << "Start training ";
			cout.flush();

			Mat original = imread("/home/glenn/ros_workspace/FGBGSeperation/Data/Original/black-smurf.jpg");
			Mat binary = imread("/home/glenn/ros_workspace/FGBGSeperation/Data/FB_image/black-smurf.jpg");
			tree.addImageToTrainingsSet(original , binary );
			original = imread("/home/glenn/ros_workspace/FGBGSeperation/Data/Original/mopper_smurf.jpg");
			binary = imread("/home/glenn/ros_workspace/FGBGSeperation/Data/FB_image/mopper_smurf.jpg");
			tree.addImageToTrainingsSet(original , binary );
			original = imread("/home/glenn/ros_workspace/FGBGSeperation/Data/Original/smurfen-salade.jpg");
			binary = imread("/home/glenn/ros_workspace/FGBGSeperation/Data/FB_image/smurfen-salade.jpg");
			tree.addImageToTrainingsSet(original , binary );
			original = imread("/home/glenn/ros_workspace/FGBGSeperation/Data/Original/smurfin.jpg");
			binary = imread("/home/glenn/ros_workspace/FGBGSeperation/Data/FB_image/smurfin.jpg");
			tree.addImageToTrainingsSet(original , binary );

			tree.train();
			tree.saveTraining("tree.xml","tree");
		}

		if(!strcmp(argv[1], "-d") || !strcmp(argv[1], "-td")){
			cout << "Start deciding ";
			cout.flush();
			Mat image = imread(argv[5]);
			Mat result = image.clone();

			if(!strcmp(argv[1], "-d")){
					tree.loadTraining("tree.xml","tree");
			}

			tree.seperateFB(image, result);

			imshow("Result", result);
			int key = -1;
			while(key == -1){
				key = waitKey(100);
			}
		}
	}catch(Exception &e){
			cout <<e.msg;

	}
	return 0;
}
