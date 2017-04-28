#ifndef MTF_PRE_PROC_H
#define MTF_PRE_PROC_H

/**
basic functions for preprocessing the raw input image
using filtering, resizing and histogram equalization
before using it for tracking
*/

#include "opencv2/imgproc/imgproc.hpp"
#include "mtf/Config/parameters.h"
#include <memory>

const vector<int> supported_output_types = { CV_32FC3, CV_32FC1, CV_8UC3, CV_8UC1 };

struct PreProcBase{

public:
	typedef std::shared_ptr<PreProcBase> Ptr;
	/**
	linked list of shared PreProc pointers to deal with heterogeneous output types
	and multiple trackers efficiently
	*/
	Ptr next;
	PreProcBase(int _output_type = CV_32FC1, double _resize_factor = 1,
		bool _hist_eq = false) : next(nullptr), output_type(_output_type),
		frame_id(-1), rgb_input(true), rgb_output(_output_type == CV_32FC3 || _output_type == CV_8UC3),
		resize_factor(_resize_factor), resize_images(_resize_factor != 1),
		hist_eq(_hist_eq){}
	virtual ~PreProcBase(){
		release();
	}
	void release(){
		frame_rgb.release();
		frame_gs.release();
		frame_rgb_uchar.release();
		if(next.get()){ next->release(); }
	}
	virtual void initialize(const cv::Mat &frame_raw, int _frame_id = -1, bool print_types = true){
		if(_frame_id > 0 && frame_id == _frame_id){
			//! this frame has already been processed
			return;
		}
		frame_id = _frame_id;
		switch(frame_raw.type()){
		case CV_8UC3:
			if(print_types){ printf("PreProc::Input type: CV_8UC3 "); }
			break;
		case CV_8UC1:
			if(print_types){ printf("PreProc::Input type: CV_8UC1 "); }
			rgb_input = false;
			break;
		default:
			throw std::invalid_argument(
				cv::format("Invalid input image type provided: %d", frame_raw.type()));
		}
		switch(output_type){
		case CV_32FC1:
			if(print_types){ printf("Output type: CV_32FC1\n"); }
			if(rgb_input){
				frame_rgb.create(frame_raw.rows, frame_raw.cols, CV_32FC3);
			}
			frame_gs.create(frame_raw.rows, frame_raw.cols, CV_32FC1);
			break;
		case CV_8UC1:
			if(print_types){ printf("Output type: CV_8UC1\n"); }
			frame_gs.create(frame_raw.rows, frame_raw.cols, CV_8UC1);
			break;
		case CV_32FC3:
			if(print_types){ printf("Output type: CV_32FC3\n"); }
			if(!rgb_input){
				frame_rgb_uchar.create(frame_raw.rows, frame_raw.cols, CV_8UC3);
			}
			frame_rgb.create(frame_raw.rows, frame_raw.cols, CV_32FC3);
			break;
		case CV_8UC3:
			if(print_types){ printf("Output type: CV_8UC3\n"); }
			frame_rgb.create(frame_raw.rows, frame_raw.cols, CV_8UC3);
			break;
		default:
			throw std::invalid_argument("PreProcBase::initialize : Invalid output image type provided");
		}
		if(resize_images){
			frame_resized.create(static_cast<int>(frame_raw.rows*resize_factor), 
				static_cast<int>(frame_raw.cols*resize_factor), output_type);
			printf("Resizing images to : %d x %d\n", frame_resized.cols, frame_resized.rows);
		}
		processFrame(frame_raw);
		if(next.get()){ next->initialize(frame_raw, _frame_id); }
	}
	virtual void update(const cv::Mat &frame_raw, int _frame_id = -1){
		if(_frame_id > 0 && frame_id == _frame_id){ return; }// this frame has already been processed
		frame_id = _frame_id;
		processFrame(frame_raw);
		if(next.get()){ next->update(frame_raw, _frame_id); }
	}
	virtual const cv::Mat& getFrame(){
		return resize_images ? frame_resized : rgb_output ? frame_rgb : frame_gs;
	}
	virtual void showFrame(std::string window_name){
		cv::Mat  disp_img;
		switch(output_type){
		case CV_32FC1:
			if(resize_images){
				disp_img.create(frame_resized.rows, frame_resized.cols, CV_8UC1);
				frame_resized.convertTo(disp_img, disp_img.type());
			} else{
				disp_img.create(frame_gs.rows, frame_gs.cols, CV_8UC1);
				frame_gs.convertTo(disp_img, disp_img.type());
			}
			imshow(window_name, disp_img);
			break;
		case CV_8UC1:
			imshow(window_name, resize_images ? frame_resized : frame_gs);
			break;
		case CV_32FC3:
			if(resize_images){
				disp_img.create(frame_resized.rows, frame_resized.cols, CV_8UC3);
				frame_resized.convertTo(disp_img, disp_img.type());
			} else{
				disp_img.create(frame_gs.rows, frame_gs.cols, CV_8UC3);
				frame_rgb.convertTo(disp_img, disp_img.type());
			}
			imshow(window_name, disp_img);
			break;
		case CV_8UC3:
			imshow(window_name, resize_images ? frame_resized : frame_rgb);
			break;
		default:
			throw std::invalid_argument("Invalid output image type provided");
		}
	}
	virtual int outputType() const{ return output_type; }
	virtual void setFrameID(int _frame_id){ frame_id = _frame_id; }
	virtual int getFrameID() const{ return frame_id; }
	virtual int getWidth() { return getFrame().cols; }
	virtual int getHeight() { return getFrame().rows; }



protected:

	void processFrame(const cv::Mat &frame_raw){
		switch(output_type){
		case CV_32FC1:
			if(rgb_input){
				frame_raw.convertTo(frame_rgb, frame_rgb.type());
				cv::cvtColor(frame_rgb, frame_gs, CV_BGR2GRAY);
			} else{
				frame_raw.convertTo(frame_gs, frame_gs.type());
			}
			if(hist_eq){
				cv::Mat frame_gs_uchar(frame_gs.rows, frame_gs.cols, CV_8UC1);
				frame_gs.convertTo(frame_gs_uchar, frame_gs_uchar.type());
				cv::equalizeHist(frame_gs_uchar, frame_gs_uchar);
				frame_gs_uchar.convertTo(frame_gs, frame_gs.type());
			}
			apply(frame_gs);
			if(resize_images){
				cv::resize(frame_gs, frame_resized, frame_resized.size());
			}
			break;
		case CV_8UC1:
			if(rgb_input){
				cv::cvtColor(frame_raw, frame_gs, CV_BGR2GRAY);
			} else{
				frame_raw.copyTo(frame_gs);
			}
			if(hist_eq){ cv::equalizeHist(frame_gs, frame_gs); }
			apply(frame_gs);
			if(resize_images){
				cv::resize(frame_gs, frame_resized, frame_resized.size());
			}
			break;
		case CV_32FC3:
			if(rgb_input){
				frame_raw.convertTo(frame_rgb, frame_rgb.type());
			} else{
				cv::cvtColor(frame_raw, frame_rgb_uchar, CV_GRAY2BGR);
				frame_rgb_uchar.convertTo(frame_rgb, frame_rgb.type());
			}
			apply(frame_rgb);
			if(resize_images){
				cv::resize(frame_rgb, frame_resized, frame_resized.size());
			}
			break;
		case CV_8UC3:
			if(rgb_input){
				frame_raw.copyTo(frame_rgb);
			} else{
				cv::cvtColor(frame_raw, frame_rgb, CV_GRAY2BGR);
			}
			apply(frame_rgb);
			if(resize_images){
				cv::resize(frame_rgb, frame_resized, frame_resized.size());
			}
			break;
		default:
			throw std::invalid_argument(
				cv::format("Invalid output image type provided: %d", output_type));
		}
	}
	virtual void apply(cv::Mat &img_gs) const = 0;

	cv::Mat frame_rgb, frame_gs, frame_rgb_uchar;
	cv::Mat frame_resized;
	int output_type;
	int frame_id;
	bool rgb_input, rgb_output;
	double resize_factor;
	bool resize_images;
	const bool hist_eq;
};

struct GaussianSmoothing : public PreProcBase{
private:
	cv::Size kernel_size;
	double sigma_x;
	double sigma_y;
public:
	GaussianSmoothing(
		int _output_type = CV_32FC1, double _resize_factor = 1, bool _hist_eq = false,
		int _kernel_size = 5,
		double _sigma_x = 3.0,
		double _sigma_y = 0) :
		PreProcBase(_output_type, _resize_factor, _hist_eq),
		kernel_size(_kernel_size, _kernel_size),
		sigma_x(_sigma_x),
		sigma_y(_sigma_y){
		printf("Using Gaussian Smoothing with ");
		printf("kernel_size: %d x %d and sigma: %f x %f\n",
			kernel_size.width, kernel_size.height, sigma_x, sigma_y);
	}
	void apply(cv::Mat &img_gs) const override{
		cv::GaussianBlur(img_gs, img_gs, kernel_size, sigma_x, sigma_y);
	}
};
struct MedianFiltering : public PreProcBase{
private:
	int kernel_size;
public:
	MedianFiltering(
		int _output_type = CV_32FC1, double _resize_factor = 1, bool _hist_eq = false,
		int _kernel_size = 5) :
		PreProcBase(_output_type, _resize_factor, _hist_eq), kernel_size(_kernel_size){
		printf("Using Median Filtering with ");
		printf("kernel_size: %d\n", kernel_size);
	}
	void apply(cv::Mat &img_gs) const override{
		cv::medianBlur(img_gs, img_gs, kernel_size);
	}
};
struct NormalizedBoxFltering : public PreProcBase{
private:
	cv::Size kernel_size;
public:
	NormalizedBoxFltering(
		int _output_type = CV_32FC1, double _resize_factor = 1, bool _hist_eq = false,
		int _kernel_size = 5) :
		PreProcBase(_output_type, _resize_factor, _hist_eq), kernel_size(_kernel_size, _kernel_size){
		printf("Using Normalized Box Fltering with ");
		printf("kernel_size: %d x %d\n", kernel_size.width, kernel_size.height);
	}
	void apply(cv::Mat &img_gs) const override{
		cv::blur(img_gs, img_gs, kernel_size);
	}
};
struct BilateralFiltering : public PreProcBase{
private:
	int diameter;
	double sigma_col;
	double sigma_space;
public:
	BilateralFiltering(
		int _output_type = CV_32FC1, double _resize_factor = 1, bool _hist_eq = false,
		int _diameter = 5,
		double _sigma_col = 15,
		double _sigma_space = 15) :
		PreProcBase(_output_type, _resize_factor, _hist_eq), diameter(_diameter),
		sigma_col(_sigma_col), sigma_space(_sigma_space){
		printf("Using Bilateral Filtering with ");
		printf("diameter: %d, sigma_col: %f and sigma_space: %f\n",
			diameter, sigma_col, sigma_space);
	}
	void apply(cv::Mat &img_gs) const override{
		// OpenCV bilateral filterig does not work in place so a copy has to be made
		cv::Mat orig_img = img_gs.clone();
		//img_gs.copyTo(orig_img);
		cv::bilateralFilter(orig_img, img_gs, diameter, sigma_col, sigma_space);
	}
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
		int _frame_id = -1, bool print_types = true) override{
		curr_frame = frame_raw;
		frame_id = _frame_id;
		output_type = curr_frame.type();
		if(print_types){
			printf("Image type : ");
			switch(output_type){
			case CV_32FC1:
				printf("CV_32FC1\n");
				break;
			case CV_32FC3:
				printf("CV_32FC3\n");
				break;
			case CV_8UC1:
				printf("CV_8UC1\n");
				break;
			case CV_8UC3:
				printf("CV_8UC3\n");
				break;
			default:
				throw std::invalid_argument(
					cv::format("Invalid image type provided: %d", output_type));
			}
		}
	}
	void update(const cv::Mat &frame_raw, int _frame_id = -1) override{
		if(frame_raw.data != curr_frame.data){
			throw std::invalid_argument("NoPreProcessing::update : Input image location in memory has changed");
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
#endif