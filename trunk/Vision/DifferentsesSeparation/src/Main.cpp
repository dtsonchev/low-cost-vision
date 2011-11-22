/*! 
 *  \brief     Test program to demonstrate the liblocator library.
 *  \details   This program is used to demonstrate a number of functions.
 *  \author    Sascha Reker
 *  \author    Glen Meerstra
 *  \author    Zep Mouris
 *  \version   1.1
 *  \date      11-10-2011
 *  \pre       OpenCV must be installed properly.
 *  \warning   Not completely tested yet. 
 *  \copyright GNU Public License.
 */

#include <iostream>
#include <cstdio>
#include <DifferensesSeperation/liblocator.h>
#include <DifferensesSeperation/cameraException.h>

using namespace std;

/*! 
 * The main function opens an connection with a webcam on video1
 * by pressing the 'b' key you take an background image
 * when an new object is introduced in the webcams line of sight the
 * detected object is calculated when the image is stable.
 * by pressing 'q' you exit the program.
 */
int main(int argc, char** argv){
	try{				
		Locator loca(0);
		int key = 0;
		cv::Mat Background;

		std::cout << "Waiting on b(ackground) || q(uit)" << std::endl;
		while (key != 'b' && key != 'q'){
			key = cv::waitKey(100);
			loca.setBackground(Background);
			cv::imshow("background",Background);
		}
		cv::Mat view, difference, blobs;

		std::cout << "Press q to quit" << std::endl;
		while(key != 'q'){
			loca.WaitForStableViewAndTakeImage(view);
			loca.showDifference(difference);
			loca.findAndDrawBlobs(blobs );
			imshow("blobs",blobs);	
			key = cv::waitKey(100);
		}
	    
	} catch(exception &e){
		cout << e.what() << endl;
	}

	return 0;
}
