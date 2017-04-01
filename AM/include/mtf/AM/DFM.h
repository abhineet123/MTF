#ifndef MTF_DFM_H
#define MTF_DFM_H

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

#include "SSDBase.h"

_MTF_BEGIN_NAMESPACE

using namespace std;
using namespace caffe;  // NOLINT(build/namespaces)
using std::string;

struct DFMParams : AMParams{
    int nfmaps;
    char* layer_name;
    int vis;
    int e_zncc;

    char *model_file_name;
    char *mean_file_name;
    char *params_file_name;

    //! value constructor
	DFMParams(const AMParams *am_params,
		int _n_fmaps, char* _layer_name, int _vis, int _zncc,
		char *_model_f_name, char *_mean_f_name, char *_params_f_name);
	//! default/copy constructor
	DFMParams(const DFMParams *params = nullptr);
};

/**
Deep Feature Maps
*/
class DFM: public SSDBase{
public:

	typedef DFMParams ParamType;     
    ParamType params; 

	DFM(const ParamType *img_params = nullptr);
 	void initializePixVals(const Matrix2Xd& curr_pts) override;
	void updatePixVals(const Matrix2Xd& curr_pts) override;
       
    std::vector<cv::Mat> extract_features(cv::Mat img, char* layer_name);
    
    std::vector<cv::Mat> init_fmaps, curr_fmaps;
    
private:

	PixValT init_pix_vals_temp, curr_pix_vals_temp;
	PixValT temp_fmaps;

	//! mean, variance and standard deviation of the initial pixel values
	double init_pix_mean, init_pix_var, init_pix_std;
	//! mean, variance and standard deviation of the current pixel values
	double curr_pix_mean, curr_pix_var, curr_pix_std;
	
	int n_pix_temp;
    boost::shared_ptr<Net<float> > net_;
    cv::Size input_geometry_;
    int num_channels_;
    cv::Mat mean_;

	void set_mean(const string& mean_file);
	void wrap_input_layer(std::vector<cv::Mat>* input_channels);
	void preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels);
	void wrap_any_layer(std::vector<cv::Mat>* input_channels, boost::shared_ptr< Blob<float> > layer);
	cv::Mat convert_float_img(cv::Mat &img);
};

_MTF_END_NAMESPACE

#endif
