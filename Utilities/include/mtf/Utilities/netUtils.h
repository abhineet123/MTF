#ifndef MTF_NET_UTILS_H
#define MTF_NET_UTILS_H

#include <caffe/layers/memory_data_layer.hpp>
#include "mtf/Macros/common.h"
#include <caffe/caffe.hpp>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <iostream>
#include "boost/scoped_ptr.hpp"
#include "caffe/util/db.hpp"
#include "caffe/util/io.hpp"
#include "mtf/SM/RegNetParams.h"

using boost::scoped_ptr;
using namespace caffe;

_MTF_BEGIN_NAMESPACE

namespace utils{
	class MTFNet{
	public:
		/**Constructs network and fills net_ object, sets mean_ , num_channels_, and input_geometry_ **/
		MTFNet(RegNetParams _rgparams);

		boost::shared_ptr<caffe::Net<float> > net_; // Network object
		boost::shared_ptr<caffe::Solver<float > > solver_;
		cv::Size input_geometry_; // W*H Input Geometry to the network
		int num_channels_; //Number of Channels
		cv::Scalar channel_mean_;

		RegNetParams rgparams;

	private:
		/** Get GPU devices in machine**/
		void getGPUs(vector<int>* gpus);

	public:
		/**preprocess the data by subtracting the mean**/
		void preprocessBatch(std::vector<cv::Mat>& batch);

		/** Extract minibatch **/
		void extractBatch(int bs, int bi, std::vector<cv::Mat> t_data, std::vector<cv::Mat> t_labels, std::vector<cv::Mat> &t_data_b, std::vector<cv::Mat> &t_labels_b);
	};

	/**Creates Network with first conv layers from conv_model till name of last_conv_layer
	 * + last fully Connected layers determined by nlayers, n_neurons and activ_fn type**/

	MTFNet *createNetwork(RegNetParams rgparams, char *conv_model = nullptr, char *last_conv_layer = nullptr,
		int n_layers = 2, int *n_neurons = nullptr, char *activ_fn = nullptr);

	/** Forward Pass the patch into the network after being resized to image_size**/
	float *forwardPass(MTFNet *network, cv::Mat patch);

	/** Train the network with training_data and labels as input **/
	void train(MTFNet *network, std::vector<cv::Mat> training_data, std::vector<cv::Mat> training_labels);

	/** Tests the Input Blobs and shows images and Label **/
	void testingInputBlobs(boost::shared_ptr<caffe::Net<float> > net_);
}

_MTF_END_NAMESPACE
#endif
