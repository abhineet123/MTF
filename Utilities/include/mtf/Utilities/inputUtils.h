#ifndef MTF_INPUT_UTILS_H
#define MTF_INPUT_UTILS_H

#include "mtf/Utilities/excpUtils.h"
#include "mtf/Macros/common.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "boost/filesystem/operations.hpp"

#ifndef DISABLE_VISP
#include <visp3/core/vpImage.h>
#include <visp3/core/vpFrameGrabber.h>
#endif
#ifndef DISABLE_XVISION
#include <XVVideo.h>
#include <XVMpeg.h>
#include <XVAVI.h>
#include <XVV4L2.h>
#include <XVDig1394.h>
#include <XVImageRGB.h>
#include <XVImageSeq.h>
#endif

using namespace std;
namespace fs = boost::filesystem;

_MTF_BEGIN_NAMESPACE
namespace utils{

	enum FrameType{ MUTABLE };

	class InputParams{
	public:
		InputParams(const InputParams *_params = nullptr);
		InputParams(char _img_source, string _dev_name, string _dev_fmt,
			string _dev_path, int _n_buffers = 1, bool _invert_seq = false);

		char img_source;
		string dev_name;
		string dev_fmt;
		string dev_path;
		int n_buffers;
		bool invert_seq;

	protected:
		void setDeafults();
	};

	class InputBase {
	public:
		InputBase(const InputParams *_params=nullptr);
		virtual ~InputBase(){}
		virtual bool initialize() = 0;
		virtual bool update() = 0;

		virtual void remapBuffer(unsigned char **new_addr) = 0;
		virtual int getFrameID() const = 0;
		virtual const cv::Mat& getFrame() const = 0;
		//! apparently a const function cannot return a non const object by reference
		virtual cv::Mat& getFrame(FrameType frame_type) = 0;
		virtual int getNFrames() const{ return n_frames; }
		virtual int getHeight() const{ return img_height; }
		virtual int getWidth() const{ return img_width; }
		/**
		return true if the memory address of the output image buffer is constant, i.e.
		the image returned by getFrame is always read into the same memory location
		*/
		virtual bool constBuffer(){ return const_buffer; }

	protected:
		int n_frames, img_width, img_height, n_channels, n_buffers, buffer_id;
		string file_path;
		bool const_buffer;
	};
	
	class InputCVParams : public InputParams {
	public:
		InputCVParams(const InputParams *_params = nullptr,
			int _img_type = CV_8UC3);
		int img_type;
	};

	class InputCV : public InputBase {
	public:
		InputCV(const InputCVParams *_params = nullptr);
		~InputCV();
		bool initialize() override;
		bool update() override;

		void remapBuffer(unsigned char** new_addr) override;
		const cv::Mat& getFrame() const override{
			return cv_buffer[buffer_id];
		}
		cv::Mat& getFrame(FrameType frame_type) override{
			return cv_buffer[buffer_id];
		}
		int getFrameID() const override{ return frame_id; }

	private:
		InputCVParams params;
		cv::VideoCapture cap_obj;
		vector<cv::Mat> cv_buffer;
		int frame_id;
	};

#ifndef DISABLE_VISP
	class InputVPParams : public InputParams {
	public:
		enum class VpResUSB{
			Default, res640x480, res800x600, res1024x768, 
			res1280x720, res1920x1080
		};
		enum class VpResFW{
			Default, res640x480, res800x600, res1024x768,
			res1280x960, res1600x1200
		};
		enum class VpDepthFW{
			Default, RGB, YUV422, Y8, Y16
		};
		enum class VpFpsUSB{
			Default, fps25, fps50
		};
		enum class VpFpsFW{
			Default, fps15, fps30, fps60, fps120, fps240,
			fps7_5, fps3_75, fps1_875
		};
		InputVPParams(const InputParams *_params,
			int _usb_n_buffers,
			VpResUSB _usb_res,
			VpFpsUSB _usb_fps,
			VpResFW _fw_res,
			VpFpsFW _fw_fps,
			VpDepthFW _pg_fw_depth,
			bool _pg_fw_print_info,
			float _pg_fw_shutter_ms,
			float _pg_fw_gain,
			float _pg_fw_exposure,
			float _pg_fw_brightness);
		InputVPParams(const InputVPParams *_params=nullptr);
		int usb_n_buffers;
		VpResUSB usb_res;
		VpFpsUSB usb_fps;
		VpResFW fw_res;
		VpFpsFW fw_fps;
		VpDepthFW fw_depth;
		bool pg_fw_print_info;
		float pg_fw_shutter_ms;
		float pg_fw_gain;
		float pg_fw_exposure;
		float pg_fw_brightness;
	};
	//! ViSP input pipeline
	class InputVP : public InputBase {
	public:
		typedef InputVPParams::VpResUSB VpResUSB;
		typedef InputVPParams::VpResFW VpResFW;
		typedef InputVPParams::VpDepthFW VpDepthPGFW;
		typedef InputVPParams::VpFpsUSB VpFpsUSB;
		typedef InputVPParams::VpFpsFW VpFpsFW;

		InputVP(const InputVPParams *_params = nullptr);
		~InputVP(){
			vp_buffer.clear();
			if(cap_obj){
				cap_obj->close();
			}
		}
		bool initialize() override;
		bool update() override;
		const cv::Mat& getFrame() const override{
			return cv_frame;
		}
		cv::Mat& getFrame(FrameType frame_type) override{
			return cv_frame;
		}
		int getFrameID() const override{ return frame_id; }

		void remapBuffer(unsigned char** new_addr) override;
		bool constBuffer() override{ return true; }

	private:
		InputVPParams params;
		typedef vpImage<vpRGBa> VPImgType;
		vector<VPImgType> vp_buffer;
		cv::Mat cv_frame;
		int frame_id;
		std::unique_ptr<vpFrameGrabber> cap_obj;

		void convert(const VPImgType &vp_img, cv::Mat &cv_img);
	};
#endif
#ifndef DISABLE_XVISION
	/*
	parameter format for XVDig1394::set_params is a string of "XnXn..."
	where X is the parameter character and n is the corresponding value
	valid characters and their values:
	B : number of buffers ( same as defined in XVVideo.h )
	R : pixel format from camera, 0 = YUV422, 1 = RGB, 2 = MONO8,
	3 = MONO16, 4 = YUV411, 5 = YUV444
	default = 0
	S : scale, 0 = any, 1 = 640x480, 2 = 320x200, 3 = 160x120,
	4 = 800x600, 5 = 1024x768, 6 = 1280x960, 7 = 1600x1200
	default = 0
	M : scale (obsolete), M0 = S4, M1 = S5, M2 = S6, M3 = S7
	C : whether to grab center of the image or not, 0 = no, 1 = yes, default = 0
	note that C1 implies format 7
	T : external trigger, 0 = off, 1-4 = mode 0-3, or directly supply the
	number to the camera. default = 0
	f : directly set IEEE 1394 camera format (as oppose to set R and S/M)
	if the value is 7, S values are still used.
	m : directly set IEEE 1394 camera mode   (as oppose to set R and S/M)
	note that this parameter should not to set for format 7
	r : frame rate, 0 = 1.875, 1 = 3.75, 2 = 7.5, 3 = 15, 4 = 30, 5 = 60 (fps)
	default = fastest under selected format and mode
	g : gain
	u : u component for the white balance
	v : v component for the white balance
	s : saturation
	A : sharpness
	h : shutter
	a : gamma
	x : exposure
	o : optical filter
	i : bus init (resets the IEEE bus)
	*/
	/*
	parameter format for XVV4L2::set_params is a string of "XnXn..."
	where X is the parameter character and n is the corresponding value
	valid characters and their values:

	I: input
	*/
	class InputXVSource{
	public:
		typedef XV_RGB24 PIX_TYPE24;
		typedef XVImageRGB< PIX_TYPE24 > IMAGE_TYPE24;
		typedef XV_RGB PIX_TYPE32;
		typedef XVImageRGB< PIX_TYPE32 > IMAGE_TYPE32;

		typedef XVVideo< IMAGE_TYPE24 > VID24;
		typedef XVVideo< IMAGE_TYPE32 > VID32;

		typedef XVV4L2< IMAGE_TYPE24 > V4L2;
		typedef XVDig1394< IMAGE_TYPE24 > DIG1394;
		typedef XVImageSeq< IMAGE_TYPE24 > IMG;
		typedef XVMpeg< IMAGE_TYPE32 >  MPG;

		InputXVSource(){}
		virtual ~InputXVSource(){}

		int img_height, img_width;
		virtual void initFrame(int) = 0;
		virtual void updateFrame(int) = 0;
		virtual void initBuffer(PIX_TYPE24**, IMAGE_TYPE24*) = 0;
		virtual void updateBuffer(PIX_TYPE24**) = 0;
	};
	class InputXV24 : public InputXVSource{
	public:
		InputXV24(char img_source,
			string dev_path, string dev_fmt, string file_path,
			int n_buffers_in, int n_frames);
		void initFrame(int init_buffer_id) override;
		void initBuffer(PIX_TYPE24 **init_addrs,
			IMAGE_TYPE24 *buffer_addrs) override;
		void updateFrame(int buffer_id) override;
		void updateBuffer(PIX_TYPE24 **new_addrs) override;
	private:
		std::unique_ptr<VID24> vid;
		IMAGE_TYPE24 *vid_buffers;
		int n_buffers;
	};
	class InputXV32 : public InputXVSource{
	public:
		InputXV32(int img_source, string file_path);
		void initFrame(int init_buffer_id) override;
		void initBuffer(PIX_TYPE24 **init_addrs,
			IMAGE_TYPE24 *buffer_addrs) override;
		// XVMpeg does not support 24 bit images so it has to be used to 
		// capture 32 bit images and then copy data to the 24 bit buffers
		void updateFrame(int buffer_id24) override;
		void updateBuffer(PIX_TYPE24 **new_addrs) override{}
		// copy image data from 32 bit source image to 24 bit buffer image
	private:
		std::unique_ptr<VID32> vid;
		IMAGE_TYPE24 *buffer24;
		int buffer_id32;
		void copyFrame32ToBuffer24(IMAGE_TYPE32 &src_frame, int buffer_id24);
	};

	class InputXV : public InputBase {

	public:
		typedef float PIX_TYPE_GS;
		typedef XV_RGB24 PIX_TYPE;
		typedef XVImageRGB<PIX_TYPE> IMAGE_TYPE;
		typedef XVImageScalar<PIX_TYPE_GS> IMAGE_TYPE_GS;

		InputXV(const InputParams *_params=nullptr) : InputBase(_params),
			params(_params), src(nullptr){}

		~InputXV(){
			cv_buffer.clear();
			xv_buffer.clear();
		}
		bool initialize() override;
		bool update() override;
		void remapBuffer(unsigned char** new_addr) override;
		const cv::Mat& getFrame() const override{
			return cv_buffer[buffer_id];
		}
		cv::Mat& getFrame(FrameType frame_type) override{
			return cv_buffer[buffer_id];
		}
		int getFrameID() const override{ return frame_id; }

	private:
		InputParams params;
		std::unique_ptr<InputXVSource> src;
		vector<IMAGE_TYPE> xv_buffer;
		vector<cv::Mat> cv_buffer;
		int frame_id;
		void copyXVToCV(IMAGE_TYPE_GS &xv_img, cv::Mat &cv_img);
		void copyXVToCVIter(IMAGE_TYPE_GS &xv_img, cv::Mat &cv_img);
		void copyXVToCVIter(IMAGE_TYPE &xv_img, cv::Mat &cv_img);
	};
#endif
	/**
	dummy input pipeline to always provide a fixed image;
	needed to ensure API uniformity of the interactive object selector
	*/
	class InputDummy : public InputBase {
		cv::Mat curr_img;
	public:
		InputDummy(const cv::Mat &img) : InputBase(), curr_img(img){}
		~InputDummy(){}
		bool initialize() override{ return true; }
		bool update() override{ return true; }
		void remapBuffer(unsigned char** new_addr) override{}
		const cv::Mat& getFrame() const override{ return curr_img; }
		cv::Mat& getFrame(FrameType) override{ return curr_img; }
		int getFrameID() const override{ return 0; }
		int getHeight() const override{ return curr_img.rows; }
		int getWidth() const override{ return curr_img.cols; }
	};
	int getNumberOfFrames(const char *file_template);
	int getNumberOfVideoFrames(const char *file_name);
}
_MTF_END_NAMESPACE
#endif
