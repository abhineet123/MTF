/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/
/*
 * TLD.cpp
 *
 *  Created on: Nov 17, 2011
 *      Author: Georg Nebehay
 */

#include "mtf/ThirdParty/TLD/TLD.h"
#include "mtf/Utilities/miscUtils.h"

#include <iostream>

#include "mtf/ThirdParty/TLD/NNClassifier.h"
#include "mtf/ThirdParty/TLD/TLDUtil.h"

using namespace std;
using namespace cv;

#define TLD_INIT_BORDER_SIZE 5

TLDParams::TLDParams(bool _trackerEnabled, bool _detectorEnabled,
	bool _learningEnabled, bool _alternating):
	trackerEnabled(_trackerEnabled),
	detectorEnabled(_detectorEnabled),
	learningEnabled(_learningEnabled),
	alternating(_alternating),
	init_border_size(TLD_INIT_BORDER_SIZE){}

TLDParams::TLDParams(const TLDParams *params) :
trackerEnabled(true),
detectorEnabled(true),
learningEnabled(true),
alternating(false),
init_border_size(TLD_INIT_BORDER_SIZE){
	if(params){
		trackerEnabled = params->trackerEnabled;
		detectorEnabled = params->detectorEnabled;
		learningEnabled = params->learningEnabled;
		alternating = params->alternating;
		init_border_size = params->init_border_size;
	}
}

namespace tld
{

	TLD::TLD() : TrackerBase()
{
	name = "tld";

	trackerEnabled = true;
    detectorEnabled = true;
    learningEnabled = true;
    alternating = false;
    valid = false;
    wasValid = false;
    learning = false;
    currBB = NULL;
    prevBB = new Rect(0,0,0,0);

    detectorCascade = new DetectorCascade();
    nnClassifier = detectorCascade->nnClassifier;

    medianFlowTracker = new MedianFlowTracker();
}
	TLD::TLD(const ParamType *tld_params) :
		TrackerBase(), params(tld_params){
		name = "tld";

		trackerEnabled = params.trackerEnabled;
		detectorEnabled = params.detectorEnabled;
		learningEnabled = params.learningEnabled;
		alternating = params.alternating;

		printf("\n");
		printf("Using TLD tracker with:\n");
		printf("trackerEnabled: %d\n", params.trackerEnabled);
		printf("detectorEnabled: %d\n", params.detectorEnabled);
		printf("learningEnabled: %d\n", params.learningEnabled);
		printf("alternating: %d\n", params.alternating);
		printf("\n");

		valid = false;
		wasValid = false;
		learning = false;
		currBB = NULL;
		prevBB = new cv::Rect(0, 0, 0, 0);
		cv_corners_mat.create(2, 4, CV_64FC1);

		detectorCascade = new DetectorCascade();
		nnClassifier = detectorCascade->nnClassifier;

		medianFlowTracker = new MedianFlowTracker();
	}

TLD::~TLD()
{
    storeCurrentData();

    if(currBB)
    {
        delete currBB;
        currBB = NULL;
    }

    if(detectorCascade)
    {
        delete detectorCascade;
        detectorCascade = NULL;
    }

    if(medianFlowTracker)
    {
        delete medianFlowTracker;
        medianFlowTracker = NULL;
    }

    if(prevBB)
    {
        delete prevBB;
        prevBB = NULL;
    }
}

void TLD::initialize(const cv::Mat& cv_corners){
	//double pos_x = (cv_corners.at<double>(0, 0) + cv_corners.at<double>(0, 1) +
	//	cv_corners.at<double>(0, 2) + cv_corners.at<double>(0, 3)) / 4;
	//double pos_y = (cv_corners.at<double>(1, 0) + cv_corners.at<double>(1, 1) +
	//	cv_corners.at<double>(1, 2) + cv_corners.at<double>(1, 3)) / 4;
	cv::Mat cv_img_gs = cv_img;
	//cv::cvtColor(cv_img, cv_img_gs, CV_BGR2GRAY);

	detectorCascade->imgWidth = cv_img_gs.cols;
	detectorCascade->imgHeight = cv_img_gs.rows;
	detectorCascade->imgWidthStep = cv_img_gs.step;

	//double pos_x = cv_corners.at<double>(0, 0);
	//double pos_y = cv_corners.at<double>(1, 0);
	//double size_x = ((cv_corners.at<double>(0, 1) - cv_corners.at<double>(0, 0)) +
	//	(cv_corners.at<double>(0, 2) - cv_corners.at<double>(0, 3))) / 2;
	//double size_y = ((cv_corners.at<double>(1, 3) - cv_corners.at<double>(1, 0)) +
	//	(cv_corners.at<double>(1, 2) - cv_corners.at<double>(1, 1))) / 2;

	cv::Rect best_fit_rect = mtf::utils::getBestFitRectangle<int>(cv_corners,
		cv_img.cols, cv_img.rows);
	//if(best_fit_rect.x + best_fit_rect.width >= cv_img_gs.cols - params.init_border_size){
	//	best_fit_rect.width = cv_img_gs.cols - params.init_border_size - best_fit_rect.x -1;
	//}
	//if(best_fit_rect.y + best_fit_rect.height >= cv_img_gs.rows - params.init_border_size){
	//	best_fit_rect.height = cv_img_gs.rows - params.init_border_size - best_fit_rect.y - 1;
	//}
	printf("best_fit_rect: x: %d y: %d width: %d height: %d\n",
		best_fit_rect.x, best_fit_rect.y, best_fit_rect.width, best_fit_rect.height);

	//printf("initialize::init_rect.x: %d\n", init_rect.x);
	//printf("initialize::init_rect.y: %d\n", init_rect.y);
	//printf("initialize::init_rect.width: %d\n", init_rect.width);
	//printf("initialize::init_rect.height: %d\n", init_rect.height);

	//printf("calling selectObject...\n");
	selectObject(cv_img_gs, &best_fit_rect);
	//printf("Done selectObject...\n");
	cv_corners.copyTo(cv_corners_mat);
	//printf("Done initialize...\n");
}

void TLD::updateCVCorners(){

	//printf("currBB->x: %d\n", currBB->x);
	//printf("currBB->y: %d\n", currBB->y);
	//printf("currBB->height: %d\n", currBB->height);
	//printf("currBB->width: %d\n", currBB->width);

	//printf("currBB: %d\n", currBB);

	double min_x, max_x, min_y, max_y;
	if(currBB){
		min_x = currBB->x;
		max_x = currBB->x + currBB->width;
		min_y = currBB->y;
		max_y = currBB->y + currBB->height;
		cv_corners_mat.at<double>(0, 0) = min_x;
		cv_corners_mat.at<double>(1, 0) = min_y;
		cv_corners_mat.at<double>(0, 1) = max_x;
		cv_corners_mat.at<double>(1, 1) = min_y;
		cv_corners_mat.at<double>(0, 2) = max_x;
		cv_corners_mat.at<double>(1, 2) = max_y;
		cv_corners_mat.at<double>(0, 3) = min_x;
		cv_corners_mat.at<double>(1, 3) = max_y;
	} else{
		//printf("TLD failed !\n");
		//cv::Rect *trackerBB = medianFlowTracker->trackerBB;
		//int numClusters = detectorCascade->detectionResult->numClusters;
		//cv::Rect *detectorBB = detectorCascade->detectionResult->detectorBB;

		//if(trackerBB){
		//	printf("trackerBB->x: %d\n", trackerBB->x);
		//	printf("trackerBB->y: %d\n", trackerBB->y);
		//	printf("trackerBB->height: %d\n", trackerBB->height);
		//	printf("trackerBB->width: %d\n", trackerBB->width);
		//} else{
		//	printf("Median Flow tracker failed !\n");
		//}
		//if(detectorBB){
		//	printf("detectorBB->x: %d\n", detectorBB->x);
		//	printf("detectorBB->y: %d\n", detectorBB->y);
		//	printf("detectorBB->height: %d\n", detectorBB->height);
		//	printf("detectorBB->width: %d\n", detectorBB->width);
		//} else{
		//	printf("Detector failed !\n");
		//}
		//printf("numClusters: %d\n", numClusters);
		//min_x = prevBB->x;
		//max_x = prevBB->x + prevBB->width;
		//min_y = prevBB->y;
		//max_y = prevBB->y + prevBB->height;
	}

	//printf("min_x: %f\n", min_x);
	//printf("max_x: %f\n", max_x);
	//printf("min_y: %f\n", min_y);
	//printf("max_y: %f\n", max_y);
}

void TLD::release()
{
    detectorCascade->release();
    medianFlowTracker->cleanPreviousData();

    if(currBB)
    {
        delete currBB;
        currBB = NULL;
    }
}

void TLD::storeCurrentData()
{
    prevImg.release();
    prevImg = currImg; //Store old image (if any)
    if(currBB)//Store old bounding box (if any)
    {
        prevBB->x = currBB->x;
        prevBB->y = currBB->y;
        prevBB->width = currBB->width;
        prevBB->height = currBB->height;
    }
    else
    {
        prevBB->x = 0;
        prevBB->y = 0;
        prevBB->width = 0;
        prevBB->height = 0;
    }

    detectorCascade->cleanPreviousData(); //Reset detector results
    medianFlowTracker->cleanPreviousData();

    wasValid = valid;
}

void TLD::selectObject(const Mat &img, Rect *bb)
{
    //Delete old object
    detectorCascade->release();

    detectorCascade->objWidth = bb->width;
    detectorCascade->objHeight = bb->height;

    //Init detector cascade
    detectorCascade->init();

    currImg = img;
    if(currBB)
    {
        delete currBB;
        currBB = NULL;
    }
    currBB = tldCopyRect(bb);
    currConf = 1;
    valid = true;

    initialLearning();

}

void TLD::processImage(const Mat &img)
{
	storeCurrentData();

	Mat grey_frame;
    //cvtColor(img, grey_frame, CV_BGR2GRAY);
	img.copyTo(grey_frame);
    currImg = grey_frame; // Store new image , right after storeCurrentData();

    if(trackerEnabled)  {
		medianFlowTracker->track(prevImg, currImg, prevBB);
	}

    if(detectorEnabled && (!alternating || medianFlowTracker->trackerBB == NULL))
    {
		detectorCascade->detect(grey_frame);

    }
    fuseHypotheses();

    learn();
}

void TLD::fuseHypotheses()
{
    Rect *trackerBB = medianFlowTracker->trackerBB;
    int numClusters = detectorCascade->detectionResult->numClusters;
    Rect *detectorBB = detectorCascade->detectionResult->detectorBB;

    if(currBB)
    {
        delete currBB;
        currBB = NULL;
    }
    currConf = 0;
    valid = false;

    float confDetector = 0;

    if(numClusters == 1)
    {
        confDetector = nnClassifier->classifyBB(currImg, detectorBB);
    }

    if(trackerBB != NULL)
    {
        float confTracker = nnClassifier->classifyBB(currImg, trackerBB);
        if(currBB)
        {
            delete currBB;
            currBB = NULL;
        }

        if(numClusters == 1 && confDetector > confTracker && tldOverlapRectRect(*trackerBB, *detectorBB) < 0.5)
        {

            currBB = tldCopyRect(detectorBB);
            currConf = confDetector;
        }
        else
        {
            currBB = tldCopyRect(trackerBB);
            currConf = confTracker;

            if(confTracker > nnClassifier->thetaTP)
            {
                valid = true;
            }
            else if(wasValid && confTracker > nnClassifier->thetaFP)
            {
                valid = true;
            }
        }
    }
    else if(numClusters == 1)
    {
        if(currBB)
        {
            delete currBB;
            currBB = NULL;
        }
        currBB = tldCopyRect(detectorBB);
        currConf = confDetector;
    }

    /*
    float var = CalculateVariance(patch.values, nn->patch_size*nn->patch_size);

    if(var < min_var) { //TODO: Think about incorporating this
        printf("%f, %f: Variance too low \n", var, classifier->min_var);
        valid = 0;
    }*/
}

void TLD::initialLearning()
{
    learning = true; //This is just for display purposes

    DetectionResult *detectionResult = detectorCascade->detectionResult;

    detectorCascade->detect(currImg);

    //This is the positive patch
    NormalizedPatch patch;
    tldExtractNormalizedPatchRect(currImg, currBB, patch.values);
    patch.positive = 1;

    float initVar = tldCalcVariance(patch.values, TLD_PATCH_SIZE * TLD_PATCH_SIZE);
    detectorCascade->varianceFilter->minVar = initVar / 2;


    float *overlap = new float[detectorCascade->numWindows];
    tldOverlapRect(detectorCascade->windows, detectorCascade->numWindows, currBB, overlap);

    //Add all bounding boxes with high overlap

    vector< pair<int, float> > positiveIndices;
    vector<int> negativeIndices;

    //First: Find overlapping positive and negative patches

    for(int i = 0; i < detectorCascade->numWindows; i++)
    {

        if(overlap[i] > 0.6)
        {
            positiveIndices.push_back(pair<int, float>(i, overlap[i]));
        }

        if(overlap[i] < 0.2)
        {
            float variance = detectionResult->variances[i];

            if(!detectorCascade->varianceFilter->enabled || variance > detectorCascade->varianceFilter->minVar)   //TODO: This check is unnecessary if minVar would be set before calling detect.
            {
                negativeIndices.push_back(i);
            }
        }
    }

    sort(positiveIndices.begin(), positiveIndices.end(), tldSortByOverlapDesc);

    vector<NormalizedPatch> patches;

    patches.push_back(patch); //Add first patch to patch list

    int numIterations = std::min<size_t>(positiveIndices.size(), 10); //Take at most 10 bounding boxes (sorted by overlap)

    for(int i = 0; i < numIterations; i++)
    {
        int idx = positiveIndices.at(i).first;
        //Learn this bounding box
        //TODO: Somewhere here image warping might be possible
        detectorCascade->ensembleClassifier->learn(&detectorCascade->windows[TLD_WINDOW_SIZE * idx], true, &detectionResult->featureVectors[detectorCascade->numTrees * idx]);
    }

    srand(1); //TODO: This is not guaranteed to affect random_shuffle

    random_shuffle(negativeIndices.begin(), negativeIndices.end());

    //Choose 100 random patches for negative examples
    for(size_t i = 0; i < std::min<size_t>(100, negativeIndices.size()); i++)
    {
        int idx = negativeIndices.at(i);

        NormalizedPatch patch;
        tldExtractNormalizedPatchBB(currImg, &detectorCascade->windows[TLD_WINDOW_SIZE * idx], patch.values);
        patch.positive = 0;
        patches.push_back(patch);
    }

    detectorCascade->nnClassifier->learn(patches);

    delete[] overlap;

}

//Do this when current trajectory is valid
void TLD::learn()
{
    if(!learningEnabled || !valid || !detectorEnabled)
    {
        learning = false;
        return;
    }

    learning = true;

    DetectionResult *detectionResult = detectorCascade->detectionResult;

    if(!detectionResult->containsValidData)
    {
        detectorCascade->detect(currImg);
    }

    //This is the positive patch
    NormalizedPatch patch;
    tldExtractNormalizedPatchRect(currImg, currBB, patch.values);

    float *overlap = new float[detectorCascade->numWindows];
    tldOverlapRect(detectorCascade->windows, detectorCascade->numWindows, currBB, overlap);

    //Add all bounding boxes with high overlap

    vector<pair<int, float> > positiveIndices;
    vector<int> negativeIndices;
    vector<int> negativeIndicesForNN;

    //First: Find overlapping positive and negative patches

    for(int i = 0; i < detectorCascade->numWindows; i++)
    {

        if(overlap[i] > 0.6)
        {
            positiveIndices.push_back(pair<int, float>(i, overlap[i]));
        }

        if(overlap[i] < 0.2)
        {
            if(!detectorCascade->ensembleClassifier->enabled || detectionResult->posteriors[i] > 0.5)   //Should be 0.5 according to the paper
            {
                negativeIndices.push_back(i);
            }

            if(!detectorCascade->ensembleClassifier->enabled || detectionResult->posteriors[i] > 0.5)
            {
                negativeIndicesForNN.push_back(i);
            }

        }
    }

    sort(positiveIndices.begin(), positiveIndices.end(), tldSortByOverlapDesc);

    vector<NormalizedPatch> patches;

    patch.positive = 1;
    patches.push_back(patch);
    //TODO: Flip


    int numIterations = std::min<size_t>(positiveIndices.size(), 10); //Take at most 10 bounding boxes (sorted by overlap)

    for(size_t i = 0; i < negativeIndices.size(); i++)
    {
        int idx = negativeIndices.at(i);
        //TODO: Somewhere here image warping might be possible
        detectorCascade->ensembleClassifier->learn(&detectorCascade->windows[TLD_WINDOW_SIZE * idx], false, &detectionResult->featureVectors[detectorCascade->numTrees * idx]);
    }

    //TODO: Randomization might be a good idea
    for(int i = 0; i < numIterations; i++)
    {
        int idx = positiveIndices.at(i).first;
        //TODO: Somewhere here image warping might be possible
        detectorCascade->ensembleClassifier->learn(&detectorCascade->windows[TLD_WINDOW_SIZE * idx], true, &detectionResult->featureVectors[detectorCascade->numTrees * idx]);
    }

    for(size_t i = 0; i < negativeIndicesForNN.size(); i++)
    {
        int idx = negativeIndicesForNN.at(i);

        NormalizedPatch patch;
        tldExtractNormalizedPatchBB(currImg, &detectorCascade->windows[TLD_WINDOW_SIZE * idx], patch.values);
        patch.positive = 0;
        patches.push_back(patch);
    }

    detectorCascade->nnClassifier->learn(patches);

    //cout << "NN has now " << detectorCascade->nnClassifier->truePositives->size() << " positives and " << detectorCascade->nnClassifier->falsePositives->size() << " negatives.\n";

    delete[] overlap;
}

typedef struct
{
    int index;
    int P;
    int N;
} TldExportEntry;

void TLD::writeToFile(const char *path)
{
    NNClassifier *nn = detectorCascade->nnClassifier;
    EnsembleClassifier *ec = detectorCascade->ensembleClassifier;

    FILE *file = fopen(path, "w");
    fprintf(file, "#Tld ModelExport\n");
    fprintf(file, "%d #width\n", detectorCascade->objWidth);
    fprintf(file, "%d #height\n", detectorCascade->objHeight);
    fprintf(file, "%f #min_var\n", detectorCascade->varianceFilter->minVar);
    fprintf(file, "%lu #Positive Sample Size\n", nn->truePositives->size());



    for(size_t s = 0; s < nn->truePositives->size(); s++)
    {
        float *imageData = nn->truePositives->at(s).values;

        for(int i = 0; i < TLD_PATCH_SIZE; i++)
        {
            for(int j = 0; j < TLD_PATCH_SIZE; j++)
            {
                fprintf(file, "%f ", imageData[i * TLD_PATCH_SIZE + j]);
            }

            fprintf(file, "\n");
        }
    }

    fprintf(file, "%lu #Negative Sample Size\n", nn->falsePositives->size());

    for(size_t s = 0; s < nn->falsePositives->size(); s++)
    {
        float *imageData = nn->falsePositives->at(s).values;

        for(int i = 0; i < TLD_PATCH_SIZE; i++)
        {
            for(int j = 0; j < TLD_PATCH_SIZE; j++)
            {
                fprintf(file, "%f ", imageData[i * TLD_PATCH_SIZE + j]);
            }

            fprintf(file, "\n");
        }
    }

    fprintf(file, "%d #numtrees\n", ec->numTrees);
    detectorCascade->numTrees = ec->numTrees;
    fprintf(file, "%d #numFeatures\n", ec->numFeatures);
    detectorCascade->numFeatures = ec->numFeatures;

    for(int i = 0; i < ec->numTrees; i++)
    {
        fprintf(file, "#Tree %d\n", i);

        for(int j = 0; j < ec->numFeatures; j++)
        {
            float *features = ec->features + 4 * ec->numFeatures * i + 4 * j;
            fprintf(file, "%f %f %f %f # Feature %d\n", features[0], features[1], features[2], features[3], j);
        }

        //Collect indices
        vector<TldExportEntry> list;

        for(int index = 0; index < pow(2.0f, ec->numFeatures); index++)
        {
            int p = ec->positives[i * ec->numIndices + index];

            if(p != 0)
            {
                TldExportEntry entry;
                entry.index = index;
                entry.P = p;
                entry.N = ec->negatives[i * ec->numIndices + index];
                list.push_back(entry);
            }
        }

        fprintf(file, "%lu #numLeaves\n", list.size());

        for(size_t j = 0; j < list.size(); j++)
        {
            TldExportEntry entry = list.at(j);
            fprintf(file, "%d %d %d\n", entry.index, entry.P, entry.N);
        }
    }

    fclose(file);

}

void TLD::readFromFile(const char *path)
{
    release();

    NNClassifier *nn = detectorCascade->nnClassifier;
    EnsembleClassifier *ec = detectorCascade->ensembleClassifier;

    FILE *file = fopen(path, "r");

    if(file == NULL)
    {
        printf("Error: Model not found: %s\n", path);
        exit(1);
    }

    int MAX_LEN = 255;
    char str_buf[255];
    fgets(str_buf, MAX_LEN, file); /*Skip line*/

    fscanf(file, "%d \n", &detectorCascade->objWidth);
    fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/
    fscanf(file, "%d \n", &detectorCascade->objHeight);
    fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

    fscanf(file, "%f \n", &detectorCascade->varianceFilter->minVar);
    fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

    int numPositivePatches;
    fscanf(file, "%d \n", &numPositivePatches);
    fgets(str_buf, MAX_LEN, file); /*Skip line*/


    for(int s = 0; s < numPositivePatches; s++)
    {
        NormalizedPatch patch;

        for(int i = 0; i < 15; i++)   //Do 15 times
        {

            fgets(str_buf, MAX_LEN, file); /*Read sample*/

            char *pch;
            pch = strtok(str_buf, " \n");
            int j = 0;

            while(pch != NULL)
            {
                float val = atof(pch);
                patch.values[i * TLD_PATCH_SIZE + j] = val;

                pch = strtok(NULL, " \n");

                j++;
            }
        }

        nn->truePositives->push_back(patch);
    }

    int numNegativePatches;
    fscanf(file, "%d \n", &numNegativePatches);
    fgets(str_buf, MAX_LEN, file); /*Skip line*/


    for(int s = 0; s < numNegativePatches; s++)
    {
        NormalizedPatch patch;

        for(int i = 0; i < 15; i++)   //Do 15 times
        {

            fgets(str_buf, MAX_LEN, file); /*Read sample*/

            char *pch;
            pch = strtok(str_buf, " \n");
            int j = 0;

            while(pch != NULL)
            {
                float val = atof(pch);
                patch.values[i * TLD_PATCH_SIZE + j] = val;

                pch = strtok(NULL, " \n");

                j++;
            }
        }

        nn->falsePositives->push_back(patch);
    }

    fscanf(file, "%d \n", &ec->numTrees);
    detectorCascade->numTrees = ec->numTrees;
    fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

    fscanf(file, "%d \n", &ec->numFeatures);
    detectorCascade->numFeatures = ec->numFeatures;
    fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

    int size = 2 * 2 * ec->numFeatures * ec->numTrees;
    ec->features = new float[size];
    ec->numIndices = pow(2.0f, ec->numFeatures);
    ec->initPosteriors();

    for(int i = 0; i < ec->numTrees; i++)
    {
        fgets(str_buf, MAX_LEN, file); /*Skip line*/

        for(int j = 0; j < ec->numFeatures; j++)
        {
            float *features = ec->features + 4 * ec->numFeatures * i + 4 * j;
            fscanf(file, "%f %f %f %f", &features[0], &features[1], &features[2], &features[3]);
            fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/
        }

        /* read number of leaves*/
        int numLeaves;
        fscanf(file, "%d \n", &numLeaves);
        fgets(str_buf, MAX_LEN, file); /*Skip rest of line*/

        for(int j = 0; j < numLeaves; j++)
        {
            TldExportEntry entry;
            fscanf(file, "%d %d %d \n", &entry.index, &entry.P, &entry.N);
            ec->updatePosterior(i, entry.index, 1, entry.P);
            ec->updatePosterior(i, entry.index, 0, entry.N);
        }
    }

    detectorCascade->initWindowsAndScales();
    detectorCascade->initWindowOffsets();

    detectorCascade->propagateMembers();

    detectorCascade->initialised = true;

    ec->initFeatureOffsets();

    fclose(file);
}


} /* namespace tld */
