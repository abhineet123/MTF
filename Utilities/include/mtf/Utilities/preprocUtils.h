#ifndef MTF_PRE_PROC_UTILS_H
#define MTF_PRE_PROC_UTILS_H

/**
basic functions for preprocessing the raw input image
using filtering, resizing and histogram equalization
before using it for tracking
*/

#include "mtf/Utilities/excpUtils.h"
#include "mtf/Utilities/imgUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <memory>

_MTF_BEGIN_NAMESPACE
namespace utils{

	const vector<int> supported_output_types = { CV_32FC3, CV_32FC1, CV_8UC3, CV_8UC1 };

	struct PreProcBase{
		typedef std::shared_ptr<PreProcBase> Ptr;
		/**
		linked list of shared PreProc pointers to deal with heterogeneous output types
		and multiple trackers efficiently
		*/
		Ptr next;
		PreProcBase(int _output_type = CV_32FC1, double _resize_factor = 1,
			bool _hist_eq = false);
		virtual ~PreProcBase(){
			release();
		}
		virtual void initialize(const cv::Mat &frame_raw, int _frame_id = -1, bool print_types = true);
		virtual void update(const cv::Mat &frame_raw, int _frame_id = -1){
			if(_frame_id > 0 && frame_id == _frame_id){ return; }// this frame has already been processed
			frame_id = _frame_id;
			processFrame(frame_raw);
			if(next.get()){ next->update(frame_raw, _frame_id); }
		}
		virtual const cv::Mat& getFrame(){
			return resize_images ? frame_resized : rgb_output ? frame_rgb : frame_gs;
		}
		virtual void showFrame(std::string window_name);
		virtual int outputType() const{ return output_type; }
		virtual void setFrameID(int _frame_id){ frame_id = _frame_id; }
		virtual int getFrameID() const{ return frame_id; }
		virtual int getWidth() { return getFrame().cols; }
		virtual int getHeight() { return getFrame().rows; }

	protected:
		cv::Mat frame_rgb, frame_gs, frame_rgb_uchar;
		cv::Mat frame_resized;
		int output_type;
		int frame_id;
		bool rgb_input, rgb_output;
		double resize_factor;
		bool resize_images;
		const bool hist_eq;

		virtual void apply(cv::Mat &img_gs) const = 0;
		virtual void processFrame(const cv::Mat &frame_raw);
		void release();
	};

	struct GaussianSmoothing : public PreProcBase{
		GaussianSmoothing(
			int _output_type = CV_32FC1, double _resize_factor = 1, bool _hist_eq = false,
			int _kernel_size = 5, double _sigma_x = 3.0, double _sigma_y = 0);
		void apply(cv::Mat &img_gs) const override{
			cv::GaussianBlur(img_gs, img_gs, kernel_size, sigma_x, sigma_y);
		}
	private:
		cv::Size kernel_size;
		double sigma_x;
		double sigma_y;
	};
	struct MedianFiltering : public PreProcBase{
		MedianFiltering(
			int _output_type = CV_32FC1, double _resize_factor = 1, bool _hist_eq = false,
			int _kernel_size = 5);
		void apply(cv::Mat &img_gs) const override{
			cv::medianBlur(img_gs, img_gs, kernel_size);
		}
	private:
		int kernel_size;
	};
	struct NormalizedBoxFltering : public PreProcBase{
		NormalizedBoxFltering(
			int _output_type = CV_32FC1, double _resize_factor = 1, bool _hist_eq = false,
			int _kernel_size = 5);
		void apply(cv::Mat &img_gs) const override{
			cv::blur(img_gs, img_gs, kernel_size);
		}
	private:
		cv::Size kernel_size;
	};
	struct BilateralFiltering : public PreProcBase{
		BilateralFiltering(
			int _output_type = CV_32FC1, double _resize_factor = 1, bool _hist_eq = false,
			int _diameter = 5, double _sigma_col = 15, double _sigma_space = 15);
		void apply(cv::Mat &img_gs) const override{
			// OpenCV bilateral filterig does not work in place so a copy has to be made
			cv::Mat orig_img = img_gs.clone();
			//img_gs.copyTo(orig_img);
			cv::bilateralFilter(orig_img, img_gs, diameter, sigma_col, sigma_space);
		}
	private:
		int diameter;
		double sigma_col;
		double sigma_space;
	};
	struct SobelFltering : public PreProcBase{
		SobelFltering(int _output_type = CV_32FC1, double _resize_factor = 1,
			bool _hist_eq = false, int _kernel_size = 3);
		void initialize(const cv::Mat &frame_raw,
			int _frame_id = -1, bool print_types = true) override;
		void processFrame(const cv::Mat &frame_raw) override;
		void apply(cv::Mat &img_gs) const override;
		const cv::Mat& getFrame() override{
			return frame_out;
		}
		void showFrame(std::string window_name) override;

	private:
		cv::Mat frame_out, frame_in;
		cv::Mat grad_x, grad_y, grad;
		cv::Mat abs_grad_x, abs_grad_y, abs_grad;
		int kernel_size;
	};
	struct AnisotropicDiffusion : public PreProcBase{
		AnisotropicDiffusion(
			int _output_type = CV_32FC1, double _resize_factor = 1, bool _hist_eq = false,
			double _lambda = 0.14285714285, double _k = 30, unsigned int _n_iters = 15);
		void apply(cv::Mat &img_gs) const override{
			mtf::utils::anisotropicDiffusion(img_gs, lambda, k, n_iters);
		}
	private:
		double lambda;
		double k;
		unsigned int n_iters;
	};
	struct NoFiltering : public PreProcBase{
		NoFiltering(int _output_type = CV_32FC1,
			double _resize_factor = 1, bool _hist_eq = false) :
			PreProcBase(_output_type, _resize_factor, _hist_eq){
			printf("Filtering is disabled\n");
		}
		void apply(cv::Mat &img_gs) const override{}
	};
	struct NoPreProcessing : public PreProcBase{
		NoPreProcessing(int _output_type = CV_32FC1) :
			PreProcBase(_output_type){
			printf("Pre processing is disabled\n");
		}
		void initialize(const cv::Mat &frame_raw,
			int _frame_id = -1, bool print_types = true) override;
		void update(const cv::Mat &frame_raw, int _frame_id = -1) override{
			if(frame_raw.data != curr_frame.data){
				throw mtf::utils::InvalidArgument("NoPreProcessing::update : Input image location in memory has changed");
			}
			frame_id = _frame_id;
		}
		void apply(cv::Mat &img_gs) const override{}
		const cv::Mat& getFrame() override{
			return curr_frame;
		}
		int outputType() const override{ return output_type; }
	private:
		cv::Mat curr_frame;
	};
}
_MTF_END_NAMESPACE
#endif