#include "mtf/Utilities/preprocUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/highgui/highgui.hpp"

_MTF_BEGIN_NAMESPACE
namespace utils{	
	PreProcBase::PreProcBase(int _output_type, double _resize_factor,
		bool _hist_eq) : next(nullptr), output_type(_output_type),
		frame_id(-1), rgb_input(true), rgb_output(_output_type == CV_32FC3 || _output_type == CV_8UC3),
		resize_factor(_resize_factor), resize_images(_resize_factor != 1),
		hist_eq(_hist_eq){}

	void PreProcBase::release(){
		frame_rgb.release();
		frame_gs.release();
		frame_rgb_uchar.release();
		if(next.get()){ next->release(); }
	}
	void PreProcBase::initialize(const cv::Mat &frame_raw, int _frame_id, bool print_types){
		if(_frame_id > 0 && frame_id == _frame_id){
			//! this frame has already been processed
			return;
		}
		frame_id = _frame_id;
		switch(frame_raw.type()){
		case CV_8UC3:
			if(print_types){ printf("Input type: CV_8UC3 "); }
			break;
		case CV_8UC1:
			if(print_types){ printf("Input type: CV_8UC1 "); }
			rgb_input = false;
			break;
		default:
			throw mtf::utils::InvalidArgument(
				cv::format("PreProcBase::initialize : Invalid input image type provided: %d", frame_raw.type()));
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
			throw mtf::utils::InvalidArgument(
				cv::format("PreProcBase::initialize : Invalid output image type provided: %d", output_type));
		}
		if(resize_images){
			frame_resized.create(static_cast<int>(frame_raw.rows*resize_factor),
				static_cast<int>(frame_raw.cols*resize_factor), output_type);
			printf("Resizing images to : %d x %d\n", frame_resized.cols, frame_resized.rows);
		}
		processFrame(frame_raw);
		if(next.get()){ next->initialize(frame_raw, _frame_id); }
	}
	void PreProcBase::showFrame(std::string window_name){
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
			throw mtf::utils::InvalidArgument(
				cv::format("PreProcBase::showFrame : Invalid output image type provided: %d", output_type));
		}
	}
	void PreProcBase::processFrame(const cv::Mat &frame_raw){
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
			throw mtf::utils::InvalidArgument(
				cv::format("PreProcBase::processFrame : Invalid output image type provided: %d", output_type));
		}
	}
	GaussianSmoothing::GaussianSmoothing(
		int _output_type, double _resize_factor, bool _hist_eq,
		int _kernel_size, double _sigma_x, double _sigma_y) :
		PreProcBase(_output_type, _resize_factor, _hist_eq),
		kernel_size(_kernel_size, _kernel_size),
		sigma_x(_sigma_x),
		sigma_y(_sigma_y){
		printf("Using Gaussian Smoothing with ");
		printf("kernel_size: %d x %d and sigma: %f x %f\n",
			kernel_size.width, kernel_size.height, sigma_x, sigma_y);
	}
	MedianFiltering::MedianFiltering( int _output_type, double _resize_factor,
		bool _hist_eq, int _kernel_size) :
		PreProcBase(_output_type, _resize_factor, _hist_eq), kernel_size(_kernel_size){
		printf("Using Median Filtering with ");
		printf("kernel_size: %d\n", kernel_size);
	}
	NormalizedBoxFltering::NormalizedBoxFltering(int _output_type , double _resize_factor,
		bool _hist_eq,	int _kernel_size) :
		PreProcBase(_output_type, _resize_factor, _hist_eq), kernel_size(_kernel_size, _kernel_size){
		printf("Using Normalized Box Fltering with ");
		printf("kernel_size: %d x %d\n", kernel_size.width, kernel_size.height);
	}
	BilateralFiltering::BilateralFiltering(	int _output_type, double _resize_factor,
		bool _hist_eq, int _diameter, double _sigma_col, double _sigma_space) :
		PreProcBase(_output_type, _resize_factor, _hist_eq), diameter(_diameter),
		sigma_col(_sigma_col), sigma_space(_sigma_space){
		printf("Using Bilateral Filtering with ");
		printf("diameter: %d, sigma_col: %f and sigma_space: %f\n",
			diameter, sigma_col, sigma_space);
	}
	SobelFltering::SobelFltering(int _output_type, double _resize_factor,
		bool _hist_eq, int _kernel_size, bool _normalize) :
		PreProcBase(_output_type, _resize_factor, _hist_eq),
		kernel_size(_kernel_size), normalize(_normalize){
		printf("Using Sobel Fltering with ");
		printf("kernel_size: %d\n", kernel_size);
		printf("normalize: %d\n", normalize);
		if(output_type == CV_8UC3 || output_type == CV_8UC1){
			throw mtf::utils::InvalidArgument(
				cv::format("SobelFltering:: Only floating point output types are supported"));
		}
	}
	void SobelFltering::initialize(const cv::Mat &frame_raw,
		int _frame_id, bool print_types){
		frame_out.create(frame_raw.rows, frame_raw.cols, output_type);
		grad_x.create(frame_raw.rows, frame_raw.cols, CV_32FC1);
		grad_y.create(frame_raw.rows, frame_raw.cols, CV_32FC1);
		if(rgb_output){
			//grad_x = cv::Mat(frame_raw.rows, frame_raw.cols, CV_32FC1, (float*)(frame_out.data), 3);
			//grad_y = cv::Mat(frame_raw.rows, frame_raw.cols, CV_32FC1, (float*)(frame_out.data + 1), 3);
			//grad = cv::Mat(frame_raw.rows, frame_raw.cols, CV_32FC1, (float*)(frame_out.data + 2), 3);
			grad.create(frame_raw.rows, frame_raw.cols, CV_32FC1);
		} else{
			grad = frame_out;
		}	
		abs_grad_x.create(frame_raw.rows, frame_raw.cols, CV_8UC1);
		abs_grad_y.create(frame_raw.rows, frame_raw.cols, CV_8UC1);
		abs_grad.create(frame_raw.rows, frame_raw.cols, CV_8UC1);
		PreProcBase::initialize(frame_raw, _frame_id, print_types);
	}
	void SobelFltering::processFrame(const cv::Mat &frame_raw){
		frame_in = frame_raw;
		if(rgb_input){
			frame_raw.convertTo(frame_rgb, frame_rgb.type());
			cv::cvtColor(frame_rgb, frame_gs, CV_BGR2GRAY);
		} else{
			frame_raw.convertTo(frame_gs, frame_gs.type());
		}
		apply(frame_gs);
	}
	void SobelFltering::apply(cv::Mat &img_gs) const{
		//printf("img_gs.type: %s\n", utils::typeToString(img_gs.type()));
		//printf("grad_x.type: %s\n", utils::typeToString(grad_x.type()));
		//printf("grad_y.type: %s\n", utils::typeToString(grad_y.type()));
		//printf("abs_grad_x.type: %s\n", utils::typeToString(abs_grad_x.type()));
		//printf("abs_grad_y.type: %s\n", utils::typeToString(abs_grad_y.type()));
		//printf("grad.type: %s\n", utils::typeToString(grad.type()));
		/// Gradient X
		Sobel(img_gs, grad_x, -1, 1, 0, kernel_size, 1, 0, cv::BORDER_DEFAULT);
		/// Gradient Y
		Sobel(img_gs, grad_y, -1, 0, 1, kernel_size, 1, 0, cv::BORDER_DEFAULT);

		if(normalize){
			cv::Mat grad_norm;
			cv::sqrt(grad_x.mul(grad_x) + grad_y.mul(grad_y), grad_norm);
			 grad_x /= grad_norm;
			 grad_y /= grad_norm;
		}

		/// Total Gradient (approximate)
		addWeighted(abs(grad_x), 0.5, abs(grad_y), 0.5, 0, grad);
		if(rgb_input){
			std::vector<cv::Mat> channels;
			channels.push_back(grad_x);
			channels.push_back(grad_y);
			channels.push_back(grad);
			merge(channels, frame_out);
		}
	}
	void SobelFltering::showFrame(std::string window_name){
		cv::Mat  disp_img;
		if(rgb_output){
			convertScaleAbs(grad_x, abs_grad_x);
			convertScaleAbs(grad_y, abs_grad_y);
			convertScaleAbs(grad, abs_grad);

			std::vector<cv::Mat> img_list;
			img_list.push_back(abs_grad_x);
			img_list.push_back(abs_grad_y);
			img_list.push_back(abs_grad);
			//imshow("abs_grad_x", abs_grad_x);
			//imshow("abs_grad_y", abs_grad_y);
			//imshow("abs_grad", abs_grad);
			cv::Mat stacked_img = utils::stackImages(img_list, 0);
			disp_img = stacked_img;
		} else{
			disp_img.create(frame_out.rows, frame_out.cols, CV_8UC1);
			frame_out.convertTo(disp_img, disp_img.type());
		}		
		imshow(window_name, disp_img);
	}
	AnisotropicDiffusion::AnisotropicDiffusion(
		int _output_type, double _resize_factor, bool _hist_eq,
		double _lambda, double _k, unsigned int _n_iters) :
		PreProcBase(_output_type, _resize_factor, _hist_eq),
		lambda(_lambda), k(_k), n_iters(_n_iters){
		printf("Using Anisotropic Diffusion with:\n");
		printf("lambda: %f\n", lambda);
		printf("k: %f\n", k);
		printf("n_iters: %d\n", n_iters);
	}
	void NoPreProcessing::initialize(const cv::Mat &frame_raw,
		int _frame_id, bool print_types){
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
				throw mtf::utils::InvalidArgument(
					cv::format("NoPreProcessing::initialize : Invalid image type provided: %d", output_type));
			}
		}
	}
}
_MTF_END_NAMESPACE

