#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/core/core.hpp>
#include <dirent.h>
#include <sstream>
#include <FGBGSeparation/FGBGSeparation.h>
#include <FGBGSeparation/variableException.h>

using namespace cv;
using namespace std;

FGBGSeparator::FGBGSeparator(int bins, int maskSize, int RGBorHSV){
	if(maskSize%2 == 0|| maskSize <= 0){
		throw variableException("maskSize needs to be odd and greater than 0");
	}

	if(bins <= 0){
		throw variableException("bins needs to be greater than 0");
	}

	if(!(RGBorHSV == RGB || RGBorHSV == HSV)){
		throw variableException("RGBorHSV needs to be 1 or 2");
	}

	this->bins = bins;
	this->maskSize = maskSize;
	this->RGBorHSV = RGBorHSV;

	trainData = Mat(bins*3, 0, CV_32FC1);
	labels = Mat(1, 0, CV_32FC1);
}

void FGBGSeparator::addImageToTrainingsSet(const Mat &image, const Mat &binaryImage){
	DataTrainer.CreateTrainDataFromImage(image, binaryImage, trainData, labels, bins, maskSize, RGBorHSV);
}

void FGBGSeparator::train(){
	float temp[] = {1, 10};
	CvDTreeParams treeParams;
	treeParams.max_depth = INT_MAX;
	treeParams.min_sample_count = 50;
	treeParams.max_categories = 2;
	treeParams.regression_accuracy = 1.0f;
	treeParams.use_surrogates = false;
	treeParams.cv_folds = 10;
	treeParams.use_1se_rule = false;
	treeParams.truncate_pruned_tree = false;
	treeParams.priors = temp;
	train(INT_MAX, 50,2,1.0f, false, 10, false,false, temp);
}

void FGBGSeparator::train(int maxDepth, int minSampleCount,int maxCategories, float regressionAccuracy, bool useSurrogates, int cvFolds, bool use1seRule, bool truncatePrunedTree, float* priors){
	CvDTreeParams treeParams;
	treeParams.max_depth = maxDepth;
	treeParams.min_sample_count = minSampleCount;
	treeParams.max_categories = maxCategories;
	treeParams.regression_accuracy = regressionAccuracy;
	treeParams.use_surrogates = useSurrogates;
	treeParams.cv_folds = cvFolds;
	treeParams.use_1se_rule = use1seRule;
	treeParams.truncate_pruned_tree = truncatePrunedTree;
	treeParams.priors = priors;
	tree.train(trainData, CV_ROW_SAMPLE, labels, Mat(), Mat(), Mat(), Mat(), treeParams);
}

void FGBGSeparator::saveTraining(const std::string& pathName, const std::string& treeName){	tree.save(pathName.c_str(), treeName.c_str());	}
void FGBGSeparator::loadTraining(const std::string& pathName, const std::string& treeName){	tree.load(pathName.c_str(), treeName.c_str());	}

void FGBGSeparator::separateFB(const Mat &image, Mat &result){
	Mat temp = image.clone();
	if(RGBorHSV == 2){
		cvtColor(image, temp, CV_RGB2HSV);
		cvtColor(result, result, CV_RGB2HSV);
	}

	MatConstIterator_<Vec3b> it = temp.begin<Vec3b>(), it_end = temp.end<Vec3b>();
	MatIterator_<Vec3b> rit = result.begin<Vec3b>();
	//for every pixel
	for (; it != it_end; ++it, ++rit)
	{
		CvDTreeNode *resultNode = tree.predict(DataTrainer.CreateHistogramFromPixel(it, bins, temp.cols, temp.rows, maskSize));
		if(resultNode->value){
			*rit = Vec3b(255,255,255);//forground
		}else{
			*rit = Vec3b(0,0,0);//background
		}
	}
	if(RGBorHSV == 2){
		cvtColor(result, result, CV_HSV2RGB);
	}
}
