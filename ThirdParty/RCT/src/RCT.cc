#include "mtf/ThirdParty/RCT/RCT.h"
#include "mtf/Utilities/miscUtils.h"
#include <math.h>
#include <iostream>

//! minimum distance of the initializing bounding box from the image corners
//! using no border causes the tracker to segfault on occasions
#define RCT_INIT_BORDER_SIZE 10
#define RCT_MIN_N_RECT 2
#define RCT_MAX_N_RECT 4
#define RCT_N_FEAT 50
#define RCT_RAD_OUTER_POS 4
#define RCT_RAD_SEARCH_WIN 25
#define RCT_LEARNING_RATE 0.85

using namespace cv;
using namespace std;

RCTParams::RCTParams(int _min_n_rect, int _max_n_rect,
	int _n_feat, int _rad_outer_pos,
	int _rad_search_win, double _learning_rate){
	min_n_rect = _min_n_rect;
	max_n_rect = _max_n_rect;
	n_feat = _n_feat;
	rad_outer_pos = _rad_outer_pos;
	rad_search_win = _rad_search_win;
	learning_rate = _learning_rate;
}
RCTParams::RCTParams(const RCTParams *params) :
min_n_rect(RCT_MIN_N_RECT),
max_n_rect(RCT_MAX_N_RECT),
n_feat(RCT_N_FEAT),
rad_outer_pos(RCT_RAD_OUTER_POS),
rad_search_win(RCT_RAD_SEARCH_WIN),
learning_rate(RCT_LEARNING_RATE){
	if(params){
		min_n_rect = params->min_n_rect;
		max_n_rect = params->max_n_rect;
		n_feat = params->n_feat;
		rad_outer_pos = params->rad_outer_pos;
		rad_search_win = params->rad_search_win;
		learning_rate = params->learning_rate;
	}
}
//------------------------------------------------

RCT::RCT(const Paramtype *rct_params) :
TrackerBase(), params(rct_params){
	name = "rct";
	cv_corners_mat.create(2, 4, CV_64FC1);

	printf("\n");
	printf("Using Realtime Compressive Tracker with:\n");
	printf("min_n_rect: %d\n", params.min_n_rect);
	printf("n_feat: %d\n", params.n_feat);
	printf("rad_outer_pos: %d\n", params.rad_outer_pos);
	printf("rad_search_win: %d\n", params.rad_search_win);
	printf("learning_rate model: %f\n", params.learning_rate);
	printf("\n");

	featureMinNumRect = params.min_n_rect;
	featureMaxNumRect = params.max_n_rect;	// number of rectangle from 2 to 4
	featureNum = params.n_feat;	// number of all weaker classifiers, i.e,feature pool
	rOuterPositive = params.rad_outer_pos;	// radical scope of positive samples
	rSearchWindow = params.rad_search_win; // size of search window
	muPositive = vector<float>(featureNum, 0.0f);
	muNegative = vector<float>(featureNum, 0.0f);
	sigmaPositive = vector<float>(featureNum, 1.0f);
	sigmaNegative = vector<float>(featureNum, 1.0f);
	learnRate = params.learning_rate;	// Learning rate parameter
}

RCT::RCT(void)
{
	featureMinNumRect = 2;
	featureMaxNumRect = 4;	// number of rectangle from 2 to 4
	featureNum = 50;	// number of all weaker classifiers, i.e,feature pool
	rOuterPositive = 4;	// radical scope of positive samples
	rSearchWindow = 25; // size of search window
	muPositive = vector<float>(featureNum, 0.0f);
	muNegative = vector<float>(featureNum, 0.0f);
	sigmaPositive = vector<float>(featureNum, 1.0f);
	sigmaNegative = vector<float>(featureNum, 1.0f);
	learnRate = 0.85f;	// Learning rate parameter
}

RCT::~RCT(void)
{
}

void RCT::initialize(const cv::Mat& init_corners){
	//double pos_x = (init_corners.at<double>(0, 0) + init_corners.at<double>(0, 1) +
	//	init_corners.at<double>(0, 2) + init_corners.at<double>(0, 3)) / 4;
	//double pos_y = (init_corners.at<double>(1, 0) + init_corners.at<double>(1, 1) +
	//	init_corners.at<double>(1, 2) + init_corners.at<double>(1, 3)) / 4;
	//cv::cvtColor(curr_img, curr_img_gs, CV_BGR2GRAY);
	//printf("initialize::img_height: %d\n", curr_img_gs.rows);
	//printf("initialize::img_width: %d\n", curr_img_gs.cols);

	//double pos_x = init_corners.at<double>(0, 0);
	//double pos_y = init_corners.at<double>(1, 0);
	////double pos_x = (init_corners.at<double>(0, 0) + init_corners.at<double>(0, 1) +
	////	init_corners.at<double>(0, 2) + init_corners.at<double>(0, 3)) / 4;
	////double pos_y = (init_corners.at<double>(1, 0) + init_corners.at<double>(1, 1) +
	////	init_corners.at<double>(1, 2) + init_corners.at<double>(1, 3)) / 4;
	//double size_x = ((init_corners.at<double>(0, 1) - init_corners.at<double>(0, 0)) +
	//	(init_corners.at<double>(0, 2) - init_corners.at<double>(0, 3))) / 2;
	//double size_y = ((init_corners.at<double>(1, 3) - init_corners.at<double>(1, 0)) +
	//	(init_corners.at<double>(1, 2) - init_corners.at<double>(1, 1))) / 2;

	//printf("initialize::pos_x: %f\n", pos_x);
	//printf("initialize::pos_y: %f\n", pos_y);
	//printf("initialize::size_x: %f\n", size_x);
	//printf("initialize::size_y: %f\n", size_y);

	//cv::Rect  init_rect(pos_x, pos_y, size_x, size_y);

	cv::Rect init_rect = mtf::utils::getBestFitRectangle<int>(init_corners);
	cv::Rect bounded_init_rect = mtf::utils::getBoundedRectangle<int>(init_rect, 
		curr_img_gs.cols, curr_img_gs.rows, RCT_INIT_BORDER_SIZE);
	cout << "init_corners:\n" << init_corners << "\n";
	printf("init_rect: x: %d y: %d width: %d height: %d\n",
		init_rect.x, init_rect.y, init_rect.width, init_rect.height);
	printf("bounded_init_rect: x: %d y: %d width: %d height: %d\n",
		bounded_init_rect.x, bounded_init_rect.y, bounded_init_rect.width, bounded_init_rect.height);
	 
	init(curr_img_gs, bounded_init_rect);
	curr_rect = bounded_init_rect;
	cv_corners_mat.create(2, 4, CV_64FC1);

	updateCVCorners();
}
void RCT::update(){
	//cv::cvtColor(curr_img, curr_img_gs, CV_BGR2GRAY);
	processFrame(curr_img_gs, curr_rect);
	updateCVCorners();
}
void RCT::updateCVCorners(){

	//printf("currBB->x: %d\n", currBB->x);
	//printf("currBB->y: %d\n", currBB->y);
	//printf("currBB->height: %d\n", currBB->height);
	//printf("currBB->width: %d\n", currBB->width);

	//printf("currBB: %d\n", currBB);

	double min_x, max_x, min_y, max_y;
	min_x = curr_rect.x;
	max_x = curr_rect.x + curr_rect.width;
	min_y = curr_rect.y;
	max_y = curr_rect.y + curr_rect.height;
	cv_corners_mat.at<double>(0, 0) = min_x;
	cv_corners_mat.at<double>(1, 0) = min_y;
	cv_corners_mat.at<double>(0, 1) = max_x;
	cv_corners_mat.at<double>(1, 1) = min_y;
	cv_corners_mat.at<double>(0, 2) = max_x;
	cv_corners_mat.at<double>(1, 2) = max_y;
	cv_corners_mat.at<double>(0, 3) = min_x;
	cv_corners_mat.at<double>(1, 3) = max_y;
	//printf("min_x: %f\n", min_x);
	//printf("max_x: %f\n", max_x);
	//printf("min_y: %f\n", min_y);
	//printf("max_y: %f\n", max_y);
}

void RCT::HaarFeature(Rect& _objectBox, int _numFeature)
/*Description: compute Haar features
  Arguments:
  -_objectBox: [x y width height] object rectangle
  -_numFeature: total number of features.The default is 50.
*/
{
	//printf("_numFeature: %d\n", _numFeature);
	features = vector< vector<Rect> >(_numFeature, vector<Rect>());
	featuresWeight = vector< vector<float> >(_numFeature, vector<float>());
	
	int numRect;
	Rect rectTemp;
	float weightTemp;
      
	for (int i=0; i<_numFeature; i++)
	{
		numRect = cvFloor(rng.uniform((double)featureMinNumRect, (double)featureMaxNumRect));
	
		for (int j=0; j<numRect; j++)
		{
			
			rectTemp.x = cvFloor(rng.uniform(0.0, (double)(_objectBox.width - 3)));
			rectTemp.y = cvFloor(rng.uniform(0.0, (double)(_objectBox.height - 3)));
			rectTemp.width = cvCeil(rng.uniform(0.0, (double)(_objectBox.width - rectTemp.x - 2)));
			rectTemp.height = cvCeil(rng.uniform(0.0, (double)(_objectBox.height - rectTemp.y - 2)));
			features[i].push_back(rectTemp);

			weightTemp = (float)pow(-1.0, cvFloor(rng.uniform(0.0, 2.0))) / sqrt(float(numRect));
			featuresWeight[i].push_back(weightTemp);
           
		}
	}
}


void RCT::sampleRect(Mat& _image, Rect& _objectBox, float _rInner, float _rOuter, int _maxSampleNum, vector<Rect>& _sampleBox)
/* Description: compute the coordinate of positive and negative sample image templates
   Arguments:
   -_image:        processing frame
   -_objectBox:    recent object position 
   -_rInner:       inner sampling radius
   -_rOuter:       Outer sampling radius
   -_maxSampleNum: maximal number of sampled images
   -_sampleBox:    Storing the rectangle coordinates of the sampled images.
*/
{
	int rowsz = _image.rows - _objectBox.height - 1;
	int colsz = _image.cols - _objectBox.width - 1;
	float inradsq = _rInner*_rInner;
	float outradsq = _rOuter*_rOuter;

  	
	int dist;

	int minrow = max(0,(int)_objectBox.y-(int)_rInner);
	int maxrow = min((int)rowsz-1,(int)_objectBox.y+(int)_rInner);
	int mincol = max(0,(int)_objectBox.x-(int)_rInner);
	int maxcol = min((int)colsz-1,(int)_objectBox.x+(int)_rInner);
    
	
	
	int i = 0;

	float prob = ((float)(_maxSampleNum))/(maxrow-minrow+1)/(maxcol-mincol+1);

	int r;
	int c;
    
    _sampleBox.clear();//important
    Rect rec(0,0,0,0);

	for( r=minrow; r<=(int)maxrow; r++ )
		for( c=mincol; c<=(int)maxcol; c++ ){
			dist = (_objectBox.y-r)*(_objectBox.y-r) + (_objectBox.x-c)*(_objectBox.x-c);

			if( rng.uniform(0.,1.)<prob && dist < inradsq && dist >= outradsq ){

                rec.x = c;
				rec.y = r;
				rec.width = _objectBox.width;
				rec.height= _objectBox.height;
				
                _sampleBox.push_back(rec);				
				
				i++;
			}
		}
	
		_sampleBox.resize(i);
		
}

void RCT::sampleRect(Mat& _image, Rect& _objectBox, float _srw, vector<Rect>& _sampleBox)
/* Description: Compute the coordinate of samples when detecting the object.*/
{
	int rowsz = _image.rows - _objectBox.height - 1;
	int colsz = _image.cols - _objectBox.width - 1;
	float inradsq = _srw*_srw;	
	

	int dist;

	int minrow = max(0,(int)_objectBox.y-(int)_srw);
	int maxrow = min((int)rowsz-1,(int)_objectBox.y+(int)_srw);
	int mincol = max(0,(int)_objectBox.x-(int)_srw);
	int maxcol = min((int)colsz-1,(int)_objectBox.x+(int)_srw);

	int i = 0;

	int r;
	int c;

	Rect rec(0,0,0,0);
    _sampleBox.clear();//important

	for( r=minrow; r<=(int)maxrow; r++ )
		for( c=mincol; c<=(int)maxcol; c++ ){
			dist = (_objectBox.y-r)*(_objectBox.y-r) + (_objectBox.x-c)*(_objectBox.x-c);

			if( dist < inradsq ){

				rec.x = c;
				rec.y = r;
				rec.width = _objectBox.width;
				rec.height= _objectBox.height;

				_sampleBox.push_back(rec);				

				i++;
			}
		}
	
		_sampleBox.resize(i);

}
// Compute the features of samples
void RCT::getFeatureValue(Mat& _imageIntegral, vector<Rect>& _sampleBox, Mat& _sampleFeatureValue)
{
	int sampleBoxSize = _sampleBox.size();
	_sampleFeatureValue.create(featureNum, sampleBoxSize, CV_32F);
	float tempValue;
	int xMin;
	int xMax;
	int yMin;
	int yMax;

	for (int i=0; i<featureNum; i++)
	{
		for (int j=0; j<sampleBoxSize; j++)
		{
			tempValue = 0.0f;
			for (size_t k=0; k<features[i].size(); k++)
			{
				xMin = _sampleBox[j].x + features[i][k].x;
				xMax = _sampleBox[j].x + features[i][k].x + features[i][k].width;
				yMin = _sampleBox[j].y + features[i][k].y;
				yMax = _sampleBox[j].y + features[i][k].y + features[i][k].height;
				tempValue += featuresWeight[i][k] * 
					(_imageIntegral.at<float>(yMin, xMin) +
					_imageIntegral.at<float>(yMax, xMax) -
					_imageIntegral.at<float>(yMin, xMax) -
					_imageIntegral.at<float>(yMax, xMin));
			}
			_sampleFeatureValue.at<float>(i,j) = tempValue;
		}
	}
}

// Update the mean and variance of the gaussian classifier
void RCT::classifierUpdate(Mat& _sampleFeatureValue, vector<float>& _mu, vector<float>& _sigma, float _learnRate)
{
	Scalar muTemp;
	Scalar sigmaTemp;
    
	for (int i=0; i<featureNum; i++)
	{
		meanStdDev(_sampleFeatureValue.row(i), muTemp, sigmaTemp);
	   
		_sigma[i] = (float)sqrt( _learnRate*_sigma[i]*_sigma[i]	+ (1.0f-_learnRate)*sigmaTemp.val[0]*sigmaTemp.val[0] 
		+ _learnRate*(1.0f-_learnRate)*(_mu[i]-muTemp.val[0])*(_mu[i]-muTemp.val[0]));	// equation 6 in paper

		_mu[i] = _mu[i]*_learnRate + (1.0f-_learnRate)*muTemp.val[0];	// equation 6 in paper
	}
}

// Compute the ratio classifier 
void RCT::radioClassifier(vector<float>& _muPos, vector<float>& _sigmaPos, vector<float>& _muNeg, vector<float>& _sigmaNeg,
										 Mat& _sampleFeatureValue, float& _radioMax, int& _radioMaxIndex)
{
	float sumRadio;
	_radioMax = -FLT_MAX;
	_radioMaxIndex = 0;
	float pPos;
	float pNeg;
	int sampleBoxNum = _sampleFeatureValue.cols;

	for (int j=0; j<sampleBoxNum; j++)
	{
		sumRadio = 0.0f;
		for (int i=0; i<featureNum; i++)
		{
			pPos = exp( (_sampleFeatureValue.at<float>(i,j)-_muPos[i])*(_sampleFeatureValue.at<float>(i,j)-_muPos[i]) / -(2.0f*_sigmaPos[i]*_sigmaPos[i]+1e-30) ) / (_sigmaPos[i]+1e-30);
			pNeg = exp( (_sampleFeatureValue.at<float>(i,j)-_muNeg[i])*(_sampleFeatureValue.at<float>(i,j)-_muNeg[i]) / -(2.0f*_sigmaNeg[i]*_sigmaNeg[i]+1e-30) ) / (_sigmaNeg[i]+1e-30);
			sumRadio += log(pPos+1e-30) - log(pNeg+1e-30);	// equation 4
		}
		if (_radioMax < sumRadio)
		{
			_radioMax = sumRadio;
			_radioMaxIndex = j;
		}
	}
}
void RCT::init(Mat& _frame, Rect& _objectBox)
{
	// compute feature template
	HaarFeature(_objectBox, featureNum);

	// compute sample templates
	sampleRect(_frame, _objectBox, rOuterPositive, 0, 1000000, samplePositiveBox);
	sampleRect(_frame, _objectBox, rSearchWindow*1.5, rOuterPositive+4.0, 100, sampleNegativeBox);

	integral(_frame, imageIntegral, CV_32F);

	getFeatureValue(imageIntegral, samplePositiveBox, samplePositiveFeatureValue);
	getFeatureValue(imageIntegral, sampleNegativeBox, sampleNegativeFeatureValue);
	classifierUpdate(samplePositiveFeatureValue, muPositive, sigmaPositive, learnRate);
	classifierUpdate(sampleNegativeFeatureValue, muNegative, sigmaNegative, learnRate);
}
void RCT::processFrame(Mat& _frame, Rect& _objectBox)
{
	// predict
	sampleRect(_frame, _objectBox, rSearchWindow,detectBox);
	integral(_frame, imageIntegral, CV_32F);
	getFeatureValue(imageIntegral, detectBox, detectFeatureValue);
	int radioMaxIndex;
	float radioMax;
	radioClassifier(muPositive, sigmaPositive, muNegative, sigmaNegative, detectFeatureValue, radioMax, radioMaxIndex);
	_objectBox = detectBox[radioMaxIndex];

	// update
	sampleRect(_frame, _objectBox, rOuterPositive, 0.0, 1000000, samplePositiveBox);
	sampleRect(_frame, _objectBox, rSearchWindow*1.5, rOuterPositive+4.0, 100, sampleNegativeBox);
	
	getFeatureValue(imageIntegral, samplePositiveBox, samplePositiveFeatureValue);
	getFeatureValue(imageIntegral, sampleNegativeBox, sampleNegativeFeatureValue);
	classifierUpdate(samplePositiveFeatureValue, muPositive, sigmaPositive, learnRate);
	classifierUpdate(sampleNegativeFeatureValue, muNegative, sigmaNegative, learnRate);
}