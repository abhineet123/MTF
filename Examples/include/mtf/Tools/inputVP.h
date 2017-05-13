#ifndef MTF_INPUT_VP
#define MTF_INPUT_VP

//#if defined _WIN32
////#define VISP_HAVE_FFMPEG
//#define VISP_HAVE_V4L2
//#define VISP_HAVE_DC1394
//#define VISP_HAVE_OPENCV
//#define VISP_HAVE_OPENCV_VERSION 0x020100
//#endif

#include "inputBase.h"
#include <visp3/core/vpImage.h>
#include <visp3/core/vpFrameGrabber.h>
#include <visp3/io/vpDiskGrabber.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

#include <memory>

enum class VpResUSB{
	Default, res640x480, res800x600, res1024x768, res1280x720, res1920x1080
};
enum class VpResFW{
	Default, res640x480, res800x600, res1024x768, res1280x960, res1600x1200
};
enum class VpFpsUSB{ Default, fps25, fps50 };
enum class VpFpsFW{ Default, fps15, fps30, fps60, fps120, fps240 };

//! ViSP input pipeline
class InputVP : public InputBase {

public:

	InputVP(char img_source, string _dev_name, string _dev_fmt,
		string _dev_path, int _n_buffers = 1, int _usb_n_buffers = 3,
		bool _invert_seq = false,
		VpResUSB _usb_res = VpResUSB::Default,
		VpFpsUSB _usb_fps = VpFpsUSB::Default,
		VpResFW _fw_res = VpResFW::Default,
		VpFpsFW _fw_fps = VpFpsFW::Default) :
		InputBase(img_source, _dev_name, _dev_fmt, _dev_path, _n_buffers, _invert_seq),
		frame_id(0), cap_obj(nullptr), usb_res(_usb_res), usb_fps(_usb_fps),
		fw_res(_fw_res), fw_fps(_fw_fps), usb_n_buffers(_usb_n_buffers){}

	~InputVP(){
		vp_buffer.clear();
		if(cap_obj){
			cap_obj->close();
		}
	}

	bool initialize() override{
		printf("Initializing ViSP pipeline...\n");
		n_channels = 3;
		if(img_source == SRC_VID || img_source == SRC_IMG) {
			vpVideoReader *vid_cap = new vpVideoReader;
			vid_cap->setFileName(file_path.c_str());
			//n_frames=vid_cap->getLastFrameIndex();
			if(img_source == SRC_VID){
				printf("Opening %s video file: %s with %d frames\n", dev_fmt.c_str(), file_path.c_str(), n_frames);
			} else{
				printf("Opening %s image files at %s with %d frames\n", dev_fmt.c_str(), file_path.c_str(), n_frames);
			}
			cap_obj.reset(vid_cap);
		} else if(img_source == SRC_DISK) {
			vpDiskGrabber *disk_cap = new vpDiskGrabber;
			disk_cap->setDirectory((dev_path + "/" + dev_name).c_str());
			disk_cap->setBaseName("frame");
			disk_cap->setStep(1);
			disk_cap->setNumberOfZero(5);
			disk_cap->setImageNumber(1);
			disk_cap->setExtension(dev_fmt.c_str());
			cap_obj.reset(disk_cap);
		}
#if defined( VISP_HAVE_V4L2 )
		else if(img_source == SRC_USB_CAM) {
			printf("Opening USB camera %s\n", dev_path.c_str());
			vpV4l2Grabber *v4l2_cap = new vpV4l2Grabber;
			v4l2_cap->setInput(atoi(dev_path.c_str()));
			v4l2_cap->setNBuffers(usb_n_buffers);
			switch(usb_res){
			case VpResUSB::Default:
				break;
			case VpResUSB::res640x480:
				v4l2_cap->setWidth(640);
				v4l2_cap->setHeight(480);
				break;
			case VpResUSB::res800x600:
				v4l2_cap->setWidth(800);
				v4l2_cap->setHeight(600);
				break;
			case VpResUSB::res1024x768:
				v4l2_cap->setWidth(1024);
				v4l2_cap->setHeight(768);
				break;
			case VpResUSB::res1280x720:
				v4l2_cap->setWidth(1280);
				v4l2_cap->setHeight(720);
				break;
			case VpResUSB::res1920x1080:
				v4l2_cap->setWidth(1920);
				v4l2_cap->setHeight(1080);
				break;
			default:
				printf("Invalid resolution provided for ViSP USB pipeline\n");
				return false;
			}
			switch(usb_fps){
			case VpFpsUSB::Default:
				break;
			case VpFpsUSB::fps25:
				v4l2_cap->setFramerate(vpV4l2Grabber::framerate_25fps);
				break;
			case VpFpsUSB::fps50:
				v4l2_cap->setFramerate(vpV4l2Grabber::framerate_50fps);
				break;
			default:
				printf("Invalid frame rate provided for ViSP USB pipeline\n");
				return false;
			}
			cap_obj.reset(v4l2_cap);
		} 
#endif
#if defined( VISP_HAVE_DC1394 )
		else if(img_source == SRC_FW_CAM) {
			vp1394TwoGrabber *dc1394_cap = new vp1394TwoGrabber;
#ifndef _WIN32
			printf("Opening FireWire camera with GUID %lu\n", dc1394_cap->getGuid());
#else
			printf("Opening FireWire camera with GUID %llu\n", dc1394_cap->getGuid());
#endif
			switch(fw_res){
			case VpResFW::Default:
				break;
			case VpResFW::res640x480:
				dc1394_cap->setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_RGB8);
				break;
			case VpResFW::res800x600:
				dc1394_cap->setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_800x600_RGB8);
				break;
			case VpResFW::res1024x768:
				dc1394_cap->setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_1024x768_RGB8);
				break;
			case VpResFW::res1280x960:
				dc1394_cap->setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_1280x960_RGB8);
				break;
			case VpResFW::res1600x1200:
				dc1394_cap->setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_1600x1200_RGB8);
				break;
			default:
				printf("Invalid resolution provided for ViSP firewire pipeline\n");
				return false;
			}
			switch(fw_fps){
			case VpFpsFW::Default:
				break;
			case VpFpsFW::fps15:
				dc1394_cap->setFramerate(vp1394TwoGrabber::vpFRAMERATE_15);
				break;
			case VpFpsFW::fps30:
				dc1394_cap->setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
				break;
			case VpFpsFW::fps60:
				dc1394_cap->setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
				break;
			case VpFpsFW::fps120:
				dc1394_cap->setFramerate(vp1394TwoGrabber::vpFRAMERATE_120);
				break;
			case VpFpsFW::fps240:
				dc1394_cap->setFramerate(vp1394TwoGrabber::vpFRAMERATE_240);
				break;
			default:
				printf("Invalid frame rate provided for ViSP firewire pipeline\n");
				return false;
			}			
			cap_obj.reset(dc1394_cap);
		}
#endif
		else {
			printf("Invalid source provided for ViSP Pipeline: %c\n", img_source);
			return false;
		}
		VPImgType temp_img;
		cap_obj->open(temp_img);

		if(cap_obj->getWidth() == 0 || cap_obj->getHeight() == 0) {
			printf("ViSP pipeline could not be initialized successfully\n");
			return false;
		} 
		img_width = cap_obj->getWidth();
		img_height = cap_obj->getHeight();
		printf("ViSP pipeline initialized successfully to grab frames of size: %d x %d\n",
			img_width, img_height);
		vp_buffer.resize(n_buffers);
		for(int i = 0; i < n_buffers; ++i){
			vp_buffer[i].init(img_height, img_width);
		}
		buffer_id = 0;
		frame_id = 0;
		if(invert_seq){
			printf("Reading sequence into buffer....\n");
			for(int i = 1; i < n_buffers; ++i){
				cap_obj->acquire(vp_buffer[n_buffers - i - 1]);
			}
			vp_buffer[n_buffers-1] = temp_img;
		} else{
			vp_buffer[buffer_id] = temp_img;
		}		
		cv_frame.create(img_height, img_width, CV_8UC3);	
#ifdef VISP_HAVE_OPENCV
		vpImageConvert::convert(vp_buffer[buffer_id], cv_frame);
#else
		convert(vp_buffer[buffer_id], cv_frame);
#endif		
		return true;
	}
	bool update() override{
		buffer_id = (buffer_id + 1) % n_buffers;
		++frame_id;
		if(!invert_seq){
			cap_obj->acquire(vp_buffer[buffer_id]);
		}		
#ifdef VISP_HAVE_OPENCV
		vpImageConvert::convert(vp_buffer[buffer_id], cv_frame);
#else
		convert(vp_buffer[buffer_id], cv_frame);
#endif
		return true;
	}
	const cv::Mat& getFrame() const override{
		return cv_frame;
	}
	cv::Mat& getFrame(FrameType frame_type) override{
		return cv_frame;
	}
	int getFrameID() const override{ return frame_id; }

	void remapBuffer(unsigned char** new_addr) override{
		for(int i = 0; i < n_buffers; ++i){
			vp_buffer[i].bitmap = (vpRGBa*)(new_addr[i]);
		}
		buffer_id = -1;
		update();
	}
	bool constBuffer() override{ return true; }

private:
	typedef vpImage<vpRGBa> VPImgType;
	vector<VPImgType> vp_buffer;
	cv::Mat cv_frame;
	int frame_id;

	std::unique_ptr<vpFrameGrabber> cap_obj;
	VpResUSB usb_res;
	VpFpsUSB usb_fps;
	VpResFW fw_res;
	VpFpsFW fw_fps;
	int usb_n_buffers;

	void convert(const VPImgType &vp_img, cv::Mat &cv_img){
		for(unsigned int row_id = 0; row_id < vp_img.getHeight(); ++row_id){
			for(unsigned int col_id = 0; col_id < vp_img.getWidth(); ++col_id){
				vpRGBa vp_val = vp_img(row_id, col_id);
				cv_img.at<cv::Vec3b>(row_id, col_id) = cv::Vec3b(vp_val.B, vp_val.G, vp_val.R);
			}
		}
	}
};

#endif



