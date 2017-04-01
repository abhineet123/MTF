/*
 * KCFTracker.h
 *      Author: Sara
 */

#ifndef MTF_KCFTRACKER_H_
#define MTF_KCFTRACKER_H_

#include <opencv2/core/core.hpp>
#include "mtf/TrackerBase.h"

struct KCFParams{
	double padding; //extra area surrounding the target
	double lambda; //regularization

	double output_sigma_factor; //spatial bandwidth (proportional to target)
	double interp_factor; //linear interpolation factor for adaptation

	double kernel_sigma; //gaussian kernel bandwidth

	//for scaling
	int number_scales;
	double scale_step;

	double scale_model_max_area;
	double scale_sigma_factor;
	double scale_learning_rate;

	bool enableScaling;
	int resize_factor;

	KCFParams(
		double _padding,
		double _lambda,
		double _output_sigma_factor,
		double _interp_factor,
		double _kernel_sigma,
		int _number_scales,
		double _scale_step,
		double _scale_model_max_area,
		double _scale_sigma_factor,
		double _scale_learning_rate,
		bool _enableScaling,
		int _resize_factor);
	KCFParams(const KCFParams *params = nullptr);
};

class KCFTracker : public mtf::TrackerBase{
public:
	cv::Mat currFrame;
	cv::Mat currCorners;

	cv::Mat convertFloatImg(cv::Mat &img);
	KCFTracker(const KCFParams *kcf_params = nullptr);
	void initialize(const cv::Mat& corners) override;
	void update() override;
	void setImage(const cv::Mat &img) override{ currFrame = img; }
	int inputType() const  override{ return CV_32FC1; }
	const cv::Mat& getRegion()  override{ return currCorners; }

private:
	struct HOGParams{
		int binSize;
		int scaleBinSize;
		int nOrients;
		int softBin;
		float clipHog;
		HOGParams()
		{
			binSize = 1;
			scaleBinSize = 4;
			nOrients = 9;
			clipHog = 0.2;
			softBin = -1;
		}
	};
	struct trackingSetup{
		cv::Mat trans_cos_win;
		cv::Mat scale_cos_win;

		cv::Mat transFourier;
		cv::Mat scaleFourier;
		cv::Mat* model_alphaf; //one 2D Mat w*h
		cv::Mat* model_xf; //nChns 2D mat w*h

		int nNumTrans;
		cv::Mat *num_trans;
		cv::Mat den_trans;
		int nNumScale;
		cv::Mat *num_scale; // w*h 1D Mat 1*numScales
		cv::Mat den_scale; // one 1D Mat 1*numScales

		double *scaleFactors;
		cv::Size scale_model_sz;

		float min_scale_factor;
		float max_scale_factor;

		float current_scale_factor;

		cv::Point centroid;
		cv::Size original;
		cv::Size padded;

	};
	KCFParams tParams;
	trackingSetup tSetup;
	HOGParams hParams;
	void train(cv::Mat img, bool first = false);
	void gaussian_correlation(cv::Mat* xf, cv::Mat* yf, int nChns, double sigma, cv::Mat & corrF);
	cv::Point displayFloat(cv::Mat img);
	void createFourier(cv::Mat original, cv::Mat& complexI, int flag = 0);
	void gaussian_shaped_labels(double sigma, int sz_w, int sz_h, cv::Mat& shiftedFilter);
	void hann(int size, cv::Mat& arr);
	float *convertTo1DFloatArray(cv::Mat &patch);
	void inverseFourier(cv::Mat original, cv::Mat& output, int flag = 0);
	cv::Mat get_scale_sample(cv::Mat img, int &nDims, bool display = false);
public:
	cv::Mat *createFeatureMap(cv::Mat& patch, int &nChns, bool isScaling = false);
	void preprocess(cv::Mat img, cv::Point centroid, int w, int h);
	cv::Rect processFrame(cv::Mat img);
	cv::Point ComputeMaxfl(cv::Mat img);
	cv::Point updateCentroid(cv::Point oldC, int w, int h, int imgw, int imgh);
};

#endif /* KCFTRACKER_H_ */
