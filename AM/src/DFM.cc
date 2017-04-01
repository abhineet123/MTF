#include "mtf/AM/DFM.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <iostream>
#include <utility>

#include "mtf/Utilities/imgUtils.h"

#define DFM_NFMAPS 10
#define DFM_LAYER "conv1"
#define DFM_VIS 0
#define DFM_EZNCC 1
//#define MODELF "../../../Googlenet_Models/deploy.prototxt"//
#define DFM_MODELF "../../../VGG_Models/VGG_deploy.prototxt"
//#define MEANF " ../../../Googlenet_Models/imagenet_mean.binaryproto"//
#define DFM_MEANF "../../../VGG_Models/VGG_mean.binaryproto"
//#define PARAMSF "../../../Googlenet_Models/imagenet_googlenet.caffemodel"//
#define DFM_PARAMSF "../../../VGG_Models/VGG_CNN_F.caffemodel" 

_MTF_BEGIN_NAMESPACE

//! value constructor
DFMParams::DFMParams(const AMParams *am_params,
int _n_fmaps, char* _layer_name, int _vis, int _zncc,
char *_model_f_name, char *_mean_f_name, 
char *_params_f_name) :
AMParams(am_params),
nfmaps(_n_fmaps),
layer_name(_layer_name),
vis(_vis),
e_zncc(_zncc),
model_file_name(_model_f_name),
mean_file_name(_mean_f_name),
params_file_name(_params_f_name){}

//! default/copy constructor
DFMParams::DFMParams(const DFMParams *params) :
AMParams(params), nfmaps(DFM_NFMAPS), layer_name(DFM_LAYER),
vis(DFM_VIS), e_zncc(DFM_EZNCC), model_file_name(DFM_MODELF),
mean_file_name(DFM_MEANF), params_file_name(DFM_PARAMSF) {
	if(params) {
		nfmaps = params->nfmaps;
		layer_name = params->layer_name;
		vis = params->vis;
		e_zncc = params->e_zncc;

		model_file_name = params->model_file_name;
		mean_file_name = params->mean_file_name;
		params_file_name = params->params_file_name;
	}
}


DFM::DFM(const ParamType *dfm_params) :
SSDBase(dfm_params), params(dfm_params) {

	printf("\n");
	printf("Using Deep Feature Maps AM with:\n");
	printf("nfmaps: %d\n", params.nfmaps);
	printf("layer_name: %s\n", params.layer_name);
	printf("vis: %d\n", params.vis);
	printf("e_zncc: %d\n", params.e_zncc);
	printf("model_file_name: %s\n", params.model_file_name);
	printf("mean_file_name: %s\n", params.mean_file_name);
	printf("params_file_name: %s\n", params.params_file_name);

	name = "dfm";

	//    n_pix= nfmaps*54*54;

	const string& model_file = params.model_file_name;
	const string& trained_file = params.params_file_name;
	const string& mean_file = params.mean_file_name;

#ifdef CPU_ONLY
	Caffe::set_mode(Caffe::CPU);
#else
	Caffe::set_mode(Caffe::GPU);
#endif

	/* Load the network. */
	net_.reset(new Net<float>(model_file, TEST));
	net_->CopyTrainedLayersFrom(trained_file);

	CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
	CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

	Blob<float>* input_layer = net_->input_blobs()[0];

	num_channels_ = input_layer->channels();
	input_layer->Reshape(1, num_channels_, 100, 100);
	net_->Reshape();
	CHECK(num_channels_ == 3 || num_channels_ == 1)
		<< "Input layer should have 1 or 3 channels.";

	/* For testing resizing the input layer instead*/
	input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
	cout << "The input Geometry is " << input_layer->width() << " " << input_layer->height() << endl;

	init_pix_mean = init_pix_var = init_pix_std = 0;
	curr_pix_mean = curr_pix_var = curr_pix_std = 0;

	const boost::shared_ptr< Blob<float> > feature_blob = net_->blob_by_name(params.layer_name);
	std::cout << "The feature blob size is " << feature_blob->shape(0) << " " << feature_blob->shape(2) << " " << feature_blob->shape(3) << std::endl;

	n_pix = feature_blob->shape(2) * feature_blob->shape(3) * params.nfmaps;

	/* Load the binaryproto mean file. */
	set_mean(mean_file);
}

cv::Mat DFM::convert_float_img(cv::Mat &img) {
	cv::Mat imgU;
	double minVal;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;
	cv::minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
	img -= minVal;
	img.convertTo(imgU, CV_8U, 255.0 / (maxVal - minVal));
	return imgU;
}

void DFM::initializePixVals(const Matrix2Xd& init_pts) {
	n_pix_temp = resx * resy;
	if(!isInitialized()->pix_vals) {
		init_pix_vals_temp.resize(n_pix_temp);
		curr_pix_vals_temp.resize(n_pix_temp);
	}
	++frame_count;

	utils::getPixVals(init_pix_vals_temp, curr_img, init_pts, n_pix_temp,
		img_height, img_width, pix_norm_mult, pix_norm_add);

	//Create the current patch
	if(!isInitialized()->pix_vals) {
		curr_pix_vals_temp = init_pix_vals_temp;
		isInitialized()->pix_vals = true;
	}
	//std::cout<<"starting a new patch"<<std::endl;
	cv::Mat init_patch(resx, resy, CV_8U);
	for(int i = 0; i < resx; i++) {
		uchar *row = init_patch.ptr(i);
		for(int j = 0; j < resy; j++) {
			row[j] = init_pix_vals_temp(i * resx + j);
		}
	}
	//imshow("testing patch", init_patch);
	//waitKey();
	//std::cout<<"Samplong Resolution "<<resx<<" "<<resy<<" n_pix "<<n_pix<<std::endl;

	//Extract feature maps of this patch
	init_fmaps = extract_features(init_patch, params.layer_name);

	//std::cout<<"size of feature maps "<<init_fmaps.size()<<" "<<init_fmaps[0].rows<<" "<<init_fmaps[0].cols<<" type "<<init_fmaps[0].type()<<std::endl;
	//n_pix= nfmaps*init_fmaps[0].rows*init_fmaps[0].cols;
	I0.resize(n_pix);
	It.resize(n_pix);

	temp_fmaps.resize(init_fmaps[0].rows * init_fmaps[0].cols);
	int c = 0;
	int c1 = 0;
	for(int m = 1; m < params.nfmaps + 1; m++) {
		for(int i = 0; i < init_fmaps[m].rows; i++) {
			float *row = init_fmaps[m].ptr<float>(i);
			for(int j = 0; j < init_fmaps[m].cols; j++) {
				temp_fmaps(c1) = row[j];
				c1++;
			}

		}

		if(params.e_zncc) {
			init_pix_mean = temp_fmaps.mean();
			temp_fmaps = (temp_fmaps.array() - init_pix_mean);
			init_pix_var = temp_fmaps.squaredNorm() / (init_fmaps[0].rows * init_fmaps[0].cols);
			init_pix_std = sqrt(init_pix_var);
			temp_fmaps /= init_pix_std;

			//pix_norm_mult = 1.0 / curr_pix_std;
			//pix_norm_add = curr_pix_mean;
		}
		c1 = 0;
		for(int i = 0; i < init_fmaps[m].rows; i++) {
			for(int j = 0; j < init_fmaps[m].cols; j++) {
				I0(c) = temp_fmaps(c1);
				c++;
				c1++;
			}
		}
		c1 = 0;
	}

	It = I0;

	std::cout << "The number of pixels is " << n_pix << std::endl;
}

void DFM::updatePixVals(const Matrix2Xd& curr_pts) {

	utils::getPixVals(curr_pix_vals_temp, curr_img, curr_pts, n_pix_temp, img_height, img_width,
		pix_norm_mult, pix_norm_add);
	//std::cout<<"starting a new patch"<<std::endl;
	cv::Mat curr_patch(resx, resy, CV_8U);
	for(int i = 0; i < resx; i++) {
		uchar *row = curr_patch.ptr(i);
		for(int j = 0; j < resy; j++) {
			row[j] = curr_pix_vals_temp(i * resx + j);
		}
	}

	curr_fmaps = extract_features(curr_patch, params.layer_name);
	int c = 0;
	int c1 = 0;
	for(int m = 1; m < params.nfmaps + 1; m++) {
		c1 = 0;
		for(int i = 0; i < curr_fmaps[m].rows; i++) {
			float *row = curr_fmaps[m].ptr<float>(i);
			for(int j = 0; j < curr_fmaps[m].cols; j++) {
				temp_fmaps(c1) = row[j];
				c1++;
			}
		}
		if(params.e_zncc) {
			curr_pix_mean = temp_fmaps.mean();
			temp_fmaps = (temp_fmaps.array() - curr_pix_mean);
			curr_pix_var = temp_fmaps.squaredNorm() / (curr_fmaps[0].rows * curr_fmaps[0].cols);
			curr_pix_std = sqrt(curr_pix_var);
			temp_fmaps /= curr_pix_std;

			pix_norm_mult = 1.0 / curr_pix_std;
			pix_norm_add = curr_pix_mean;
		}
		c1 = 0;
		for(int i = 0; i < curr_fmaps[m].rows; i++) {
			for(int j = 0; j < curr_fmaps[m].cols; j++) {
				It(c) = temp_fmaps(c1);
				c++;
				c1++;
			}
		}
		c1 = 0;
	}
}

/* Load the mean file in binaryproto format. */
void DFM::set_mean(const string& mean_file) {
	BlobProto blob_proto;
	ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

	/* Convert from BlobProto to Blob<float> */
	Blob<float> mean_blob;
	mean_blob.FromProto(blob_proto);
	CHECK_EQ(mean_blob.channels(), num_channels_)
		<< "Number of channels of mean file doesn't match input layer.";

	/* The format of the mean file is planar 32-bit float BGR or grayscale. */
	std::vector<cv::Mat> channels;
	float* data = mean_blob.mutable_cpu_data();
	for(int i = 0; i < num_channels_; ++i) {
		/* Extract an individual channel. */
		cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
		channels.push_back(channel);
		data += mean_blob.height() * mean_blob.width();
	}

	/* Merge the separate channels into a single image. */
	cv::Mat mean;
	cv::merge(channels, mean);

	/* Compute the global mean pixel value and create a mean image
	 * filled with this value. */
	cv::Scalar channel_mean = cv::mean(mean);
	channel_mean[0] = (channel_mean[0] + channel_mean[1] + channel_mean[2]) / (float)3;
	channel_mean[1] = channel_mean[0];
	channel_mean[2] = channel_mean[0];
	cout << "Channel mean is " << channel_mean << endl;
	mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);
	//  mean_ = cv::Mat(input_geometry_, CV_32FC1, channel_mean);
}

void DFM::wrap_any_layer(std::vector<cv::Mat>* input_channels, boost::shared_ptr<Blob<float> > layer) {
	int width = layer->width();
	int height = layer->height();
	float* input_data = layer->mutable_cpu_data();
	for(int i = 0; i < layer->channels(); ++i) {
		cv::Mat channel(height, width, CV_32FC1, input_data);
		input_channels->push_back(channel);
		input_data += width * height;
	}
}

std::vector<cv::Mat> DFM::extract_features(cv::Mat img, char* l_name) {
	Blob<float>* input_layer = net_->input_blobs()[0];
	input_layer->Reshape(1, num_channels_,
		input_geometry_.height, input_geometry_.width);
	/* Forward dimension change to all layers. */
	net_->Reshape();

	std::vector<cv::Mat> input_channels;
	wrap_input_layer(&input_channels);

	preprocess(img, &input_channels);

	net_->ForwardPrefilled();

	const boost::shared_ptr< Blob<float> > feature_blob = net_->blob_by_name(l_name);
	//std::cout<<"Extracted feature map from conv2 , size: ";
	//vector<int> v= feature_blob->shape();
	//for(int i=0; i<v.size(); i++)
	//cout<<v[i]<<" ";
	//cout<<endl;
	//cout<<"W H "<<feature_blob->width()<<" "<<feature_blob->height()<<endl;
	std::vector<cv::Mat> fmap;
	wrap_any_layer(&fmap, feature_blob);
	if(params.vis) {
		for(int i = 0; i < params.nfmaps; i++) {
			cv::Mat mp = convert_float_img(fmap[i]);
			std::cout << "Showing feature Map " << i << mp.rows << " " << mp.cols << endl;
			cv::imshow("Feature Map", mp);
			cv::waitKey();
		}
	}
	return fmap;
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void DFM::wrap_input_layer(std::vector<cv::Mat>* input_channels) {
	Blob<float>* input_layer = net_->input_blobs()[0];

	int width = input_layer->width();
	int height = input_layer->height();
	float* input_data = input_layer->mutable_cpu_data();
	for(int i = 0; i < input_layer->channels(); ++i) {
		cv::Mat channel(height, width, CV_32FC1, input_data);
		input_channels->push_back(channel);
		input_data += width * height;
	}
}

void DFM::preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels) {
	/* Convert the input image to the input image format of the network. */
	cv::Mat sample;
	if(img.channels() == 3 && num_channels_ == 1)
		cv::cvtColor(img, sample, CV_BGR2GRAY);
	else if(img.channels() == 4 && num_channels_ == 1)
		cv::cvtColor(img, sample, CV_BGRA2GRAY);
	else if(img.channels() == 4 && num_channels_ == 3)
		cv::cvtColor(img, sample, CV_BGRA2BGR);
	else if(img.channels() == 1 && num_channels_ == 3)
		cv::cvtColor(img, sample, CV_GRAY2BGR);
	else
		sample = img;

	cv::Mat sample_resized;
	if(sample.size() != input_geometry_)
		cv::resize(sample, sample_resized, input_geometry_);
	else
		sample_resized = sample;

	cv::Mat sample_float;
	if(num_channels_ == 3)
		sample_resized.convertTo(sample_float, CV_32FC3);
	else
		sample_resized.convertTo(sample_float, CV_32FC1);

	cv::Mat sample_normalized;
	cv::subtract(sample_float, mean_, sample_normalized);
	//sample_normalized= sample_float;

	/* This operation will write the separate BGR planes directly to the
	 * input layer of the network because it is wrapped by the cv::Mat
	 * objects in input_channels. */
	cv::split(sample_normalized, *input_channels);

	// CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
	//     == net_->input_blobs()[0]->cpu_data())
	// << "Input channels are not wrapping the input layer of the network.";
}

_MTF_END_NAMESPACE
