#ifndef INC_Params_H
#define INC_Params_H

#include <iostream>


struct HOGParams
{
	int binSize;
	int nOrients;
	int softBin;
	float clipHog;
};

struct GT
{
	double tlx_;
	double tly_;
	double trx_;
	double try_;
	double blx_;
	double bly_;
	double brx_;
	double bry_;
};

struct target
{
	CvRect init;
	int firstFrame;

	target(int x, int y, int w, int h, int firstF)
	{
		init.x= x;
		init.y= y;
		init.width= w;
		init.height= h;
		firstFrame= firstF;
	}
};

struct trackingSetup
{
    cv::Mat init_bb;
	cv::Mat trans_cos_win;
	cv::Mat scale_cos_win;
	cv::Mat rot_cos_win;

	cv::Mat transFourier;
	cv::Mat scaleFourier;
	cv::Mat rotFourier;

	int nNumTrans;
	cv::Mat *num_trans;
	cv::Mat den_trans;
	int nNumScale;
	cv::Mat *num_scale;
	cv::Mat den_scale;
	int nNumRot;
	cv::Mat *num_rot;
	cv::Mat den_rot;

	double *scaleFactors;
	cv::Size scale_model_sz;

    double *rotFactors;

	float min_scale_factor;
	float max_scale_factor;

	float current_scale_factor;
	float current_rot_factor;

	cv::Point centroid;
	cv::Size original;
    float original_rot;
	cv::Size padded;
    cv::RotatedRect current_bb;    
};

struct DSSTParams
{
	double padding;
	double output_sigma_factor;
	double scale_sigma_factor;
	double lambda;
	double learning_rate;
	double const_learning_rate;
	int number_scales;
	int number_rots;
	double scale_step;
	double rot_step;
	int scale_model_max_area;
	int resize_factor;
	int is_scaling;
	int is_rotating;
	int bin_size;

	DSSTParams(double padding_, double output_sigma_factor_, 
		double scale_sigma_factor_, double lambda_,
		double learning_rate_, int number_scales_, 
		double scale_step_, int number_rots,
		double rot_step, int resize_factor_,
		int is_scaling_, int is_rotating_, int bin_size_);
	DSSTParams(const DSSTParams *params = nullptr);

};

#endif
