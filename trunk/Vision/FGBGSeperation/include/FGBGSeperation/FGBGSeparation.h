#ifndef DECISIONTREE_H_
#define DECISIONTREE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <sstream>
#include "TrainingData.h"

/**
 * @brief the class were an decision tree is trained for fore- and background separation
 * @author Zep
 * @author Glenn
 * @version 3.0
 * @date 10-2011
 */
class FGBGSeparator{
public:
	/**
	 * @brief constructor
	 * @param bins the amount of bins where the values are divided over
	 * @param maskSize the size of the mask at which the histograms are made
	 * @param RGBorHSV 1 for RGB and 2 for HSV
	 */
	FGBGSeparator(int bins, int maskSize, int RGBorHSV);
    
	/**
	 * @brief function for adding a image to the trainingsSet
	 * @param image the total image
	 * @param binaryImage a binarie image of the normal image where white means forground and black means background
	 */
	void addImageToTrainingsSet(Mat &image, Mat &binaryImage);

	/**
	 * @brief trains a decision tree for background foreground seperation whit out parameters
	 */
	void train();
	/**
	* @brief trains a decision tree for background foreground seperation
	* @param maxDepth The maximum possible depth of the tree. That is the training algorithms attempts to split a node while its depth is less than max_depth. The actual depth may be smaller if the other termination criteria are met (see the outline of the training procedure in the beginning of the section), and/or if the tree is pruned.
	* @param minSampleCount If the number of samples in a node is less than this parameter then the node will not be splitted.
	* @param regressionAccuracy Termination criteria for regression trees. If all absolute differences between an estimated value in a node and values of train samples in this node are less than this parameter then the node will not be splitted.
	* @param useSurrogates If true then surrogate splits will be built. These splits allow to work with missing data and compute variable importance correctly.
	* @param maxCategories Cluster possible values of a categorical variable into K  max_categories clusters to find a suboptimal split. If a discrete variable, on which the training procedure tries to make a split, takes more than max_categories values, the precise best subset estimation may take a very long time because the algorithm is exponential. Instead, many decision trees engines (including ML) try to find sub-optimal split in this case by clustering all the samples into max_categories clusters that is some categories are merged together. The clustering is applied only in n>2-class classification problems for categorical variables with N > max_categories possible values. In case of regression and 2-class classification the optimal split can be found efficiently without employing clustering, thus the parameter is not used in these cases.
	* @param cvFolds If cv_folds > 1 then prune a tree with K-fold cross-validation where K is equal to cv_folds.
	* @param use1seRule If true then a pruning will be harsher. This will make a tree more compact and more resistant to the training data noise but a bit less accurate.
	* @param truncatePrunedTree If true then pruned branches are physically removed from the tree. Otherwise they are retained and it is possible to get results from the original unpruned (or pruned less aggressively) tree by decreasing CvDTree::pruned_tree_idx parameter.
	* @param priors The array of a priori class probabilities, sorted by the class label value. The parameter can be used to tune the decision tree preferences toward a certain class. For example, if you want to detect some rare anomaly occurrence, the training base will likely contain much more normal cases than anomalies, so a very good classification performance will be achieved just by considering every case as normal. To avoid this, the priors can be specified, where the anomaly probability is artificially increased (up to 0.5 or even greater), so the weight of the misclassified anomalies becomes much bigger, and the tree is adjusted properly. You can also think about this parameter as weights of prediction categories which determine relative weights that you give to misclassification. That is, if the weight of the first category is 1 and the weight of the second category is 10, then each mistake in predicting the second category is equivalent to making 10 mistakes in predicting the first category.
	*/
	void train(int maxDepth, int minSampleCount,int maxCategories, float regressionAccuracy, bool useSurrogates, int cvFolds, bool use1seRule, bool truncatePrunedTree, float* priors);

	/**
	 * @brief decides for each pixel in the image if its fore or background
	 * @param image the image on which is checked if its fore or background
	 * @param result an matrix in which the result is placed white means forground back means background
	 */
	void seperateFB(const Mat &image, Mat &result);

	/**
	 * @brief save the tree
	 * @param pathName the path were it need to be saved to
	 * @param treeName the name at which the tree is saved in the .xml
	 */
	void saveTraining(const char* pathName, const char* treeName);

	/**
	 * @brief loads the tree
	 * @param pathName the path were its saved to
	 * @param treeName the name at which the tree is loaded from the .xml
	 */
	void loadTraining(const char* pathName, const char* treeName);

private:

	///@brief the decision tree
	CvDTree tree;
	///@brief 1 for RGB and 2 for HSV
	int RGBorHSV;

	///@brief bins the amount of bins where the values are divided over
	int bins;
	///@brief maskSize the size of the mask at which the histograms are made
	int maskSize;
	///@brief constructor from dataTrainer
	Trainer DataTrainer;
	///@brief matrix whit the training data
	Mat trainData;
	///@brief matrix whit the labels of the training data
	Mat labels;

};

#endif /*DECISIONTREE_H_*/
