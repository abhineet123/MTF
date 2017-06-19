/************************************************************************
* File:	CompressiveTracker.h - renamed to RCT.h for MTF
* Brief: C++ demo for paper: Kaihua Zhang, Lei Zhang, Ming-Hsuan Yang,"Real-Time Compressive Tracking," ECCV 2012.
* Version: 1.0
* Author: Yang Xian
* Email: yang_xian521@163.com
* Date:	2012/08/03
* History:
* Revised by Kaihua Zhang on 14/8/2012, 23/8/2012
* Email: zhkhua@gmail.com
* Homepage: http://www4.comp.polyu.edu.hk/~cskhzhang/
* Project Website: http://www4.comp.polyu.edu.hk/~cslzhang/CT/CT.htm
************************************************************************/
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "mtf/TrackerBase.h"

using std::vector;

struct RCTParams{
	int min_n_rect;
	int max_n_rect;
	int n_feat;
	int rad_outer_pos;
	int rad_search_win;
	double learning_rate;

	RCTParams(int _min_n_rect, int _max_n_rect,
		int _n_feat, int _rad_outer_pos,
		int _rad_search_win, double _learning_rate);
	RCTParams(const RCTParams *params = nullptr);
};

/**
Realtime Compressive Tracker
*/
class RCT : public mtf::TrackerBase
{
public:
	typedef RCTParams Paramtype;
	~RCT(void);
	RCT(const Paramtype *rct_params=nullptr);

private:
	Paramtype params;

	int featureMinNumRect;
	int featureMaxNumRect;
	int featureNum;
	vector< vector<cv::Rect > > features;
	vector< vector<float> > featuresWeight;
	int rOuterPositive;
	vector<cv::Rect > samplePositiveBox;
	vector<cv::Rect > sampleNegativeBox;
	int rSearchWindow;
	cv::Mat imageIntegral;
	cv::Mat samplePositiveFeatureValue;
	cv::Mat sampleNegativeFeatureValue;
	vector<float> muPositive;
	vector<float> sigmaPositive;
	vector<float> muNegative;
	vector<float> sigmaNegative;
	float learnRate;
	vector<cv::Rect > detectBox;
	cv::Mat detectFeatureValue;
	cv::RNG rng;
	cv::Mat curr_img_gs;
	cv::Rect  curr_rect;
private:
	void HaarFeature(cv::Rect & _objectBox, int _numFeature);
	void sampleRect(cv::Mat& _image, cv::Rect & _objectBox, float _rInner, float _rOuter, int _maxSampleNum, vector<cv::Rect >& _sampleBox);
	void sampleRect(cv::Mat& _image, cv::Rect & _objectBox, float _srw, vector<cv::Rect >& _sampleBox);
	void getFeatureValue(cv::Mat& _imageIntegral, vector<cv::Rect >& _sampleBox, cv::Mat& _sampleFeatureValue);
	void classifierUpdate(cv::Mat& _sampleFeatureValue, vector<float>& _mu, vector<float>& _sigma, float _learnRate);
	void radioClassifier(vector<float>& _muPos, vector<float>& _sigmaPos, vector<float>& _muNeg, vector<float>& _sigmaNeg,
		cv::Mat& _sampleFeatureValue, float& _radioMax, int& _radioMaxIndex);
public:
	void processFrame(cv::Mat& _frame, cv::Rect & _objectBox);
	void init(cv::Mat& _frame, cv::Rect & _objectBox);

	void initialize(const cv::Mat& init_corners) override;
	void update() override;
	void setImage(const cv::Mat &img) override{ curr_img_gs = img; }
	int inputType() const  override{ return CV_8UC1; }
	const cv::Mat& getRegion() override{ return cv_corners_mat; }
	void updateCVCorners();

};
