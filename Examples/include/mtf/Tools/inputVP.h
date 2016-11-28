#ifndef MTF_INPUT_VP
#define MTF_INPUT_VP

#if defined _WIN32
//#define VISP_HAVE_FFMPEG
#define VISP_HAVE_V4L2
#define VISP_HAVE_DC1394
#define VISP_HAVE_OPENCV
#define VISP_HAVE_OPENCV_VERSION 0x020100
#endif

#include "inputBase.h"
#include <visp3/core/vpImage.h>
#include <visp3/core/vpFrameGrabber.h>
#include <visp3/io/vpDiskGrabber.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

enum class VpResUSB{
	Default, res640x480, res800x600, res1024x768, res1280x720, res1920x1080
};
enum class VpResFW{
	Default, res640x480, res800x600, res1024x768, res1280x960, res1600x1200
};
enum class VpFpsUSB{ Default, fps25, fps50 };
enum class VpFpsFW{ Default, fps15, fps30, fps60, fps120, fps240 };

// ViSP input pipeline
class InputVP : public InputBase {
	typedef vpImage<vpRGBa> VPImgType;
	vector<VPImgType> vp_buffer;
	cv::Mat cv_frame;

	vpFrameGrabber *cap_obj;
	VpResUSB usb_res;
	VpFpsUSB usb_fps;
	VpResFW fw_res;
	VpFpsFW fw_fps;
	int usb_n_buffers;

	int frame_id;

public:

	InputVP(char img_source, string dev_name_in, string dev_fmt_in,
		string dev_path_in, int n_buffers = 1, int _usb_n_buffers=3,
		VpResUSB _usb_res = VpResUSB::Default,
		VpFpsUSB _usb_fps = VpFpsUSB::Default,
		VpResFW _fw_res = VpResFW::Default,
		VpFpsFW _fw_fps = VpFpsFW::Default) :
		InputBase(img_source, dev_name_in, dev_fmt_in, dev_path_in, n_buffers),
		cap_obj(nullptr), usb_res(_usb_res), usb_fps(_usb_fps),
		fw_res(_fw_res), fw_fps(_fw_fps), frame_id(0), usb_n_buffers(_usb_n_buffers){}
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
			cap_obj = vid_cap;
		} else if(img_source == SRC_DISK) {
			vpDiskGrabber *disk_cap = new vpDiskGrabber;
			disk_cap->setDirectory((dev_path + "/" + dev_name).c_str());
			disk_cap->setBaseName("frame");
			disk_cap->setStep(1);
			disk_cap->setNumberOfZero(5);
			disk_cap->setImageNumber(1);
			disk_cap->setExtension(dev_fmt.c_str());
			cap_obj = disk_cap;
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
			cap_obj = v4l2_cap;
		} 
#endif
#if defined( VISP_HAVE_DC1394 )
		else if(img_source == SRC_FW_CAM) {
			vp1394TwoGrabber *dc1394_cap = new vp1394TwoGrabber;
			printf("Opening FireWire camera with GUID %llu\n", dc1394_cap->getGuid());
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
			cap_obj = dc1394_cap;
		}
#endif
		else {
			std::cout << "==========Invalid source provided for ViSP Pipeline==========\n";
			return false;
		}
		VPImgType temp_img;
		cap_obj->open(temp_img);

		if(cap_obj->getWidth() == 0 || cap_obj->getHeight() == 0) {
			printf("ViSP pipeline could not be initialized successfully\n");
			return false;
		} else{
			img_width = cap_obj->getWidth();
			img_height = cap_obj->getHeight();
			printf("ViSP pipeline initialized successfully to grab frames of size: %d x %d\n", 
				img_width, img_height);
		}

		vp_buffer.resize(n_buffers);
		for(int i = 0; i < n_buffers; ++i){
			vp_buffer[i].init(img_height, img_width);
		}
		cv_frame.create(img_height, img_width, CV_8UC3);
		buffer_id = 0;
		vp_buffer[buffer_id] = temp_img;
		vpImageConvert::convert(vp_buffer[buffer_id], cv_frame);
		frame_id = 0;
		return true;
	}

	const cv::Mat& getFrame() const override{
		return cv_frame;
	}
	cv::Mat& getFrame(FrameType frame_type) override{
		return cv_frame;
	}
	bool update() override{
		buffer_id = (buffer_id + 1) % n_buffers;
		++frame_id;
		cap_obj->acquire(vp_buffer[buffer_id]);
		vpImageConvert::convert(vp_buffer[buffer_id], cv_frame);
		return true;		
	}
	int getFrameID() const override{ return frame_id; }

	void remapBuffer(uchar** new_addr) override{
		for(int i = 0; i < n_buffers; i++){
			//printf("Remapping CV buffer %d to: %lu\n", i, (unsigned long)new_addr[i]);
			vp_buffer[i].bitmap = (vpRGBa*) (new_addr[i]);
		}
		buffer_id = -1;
		update();
	}
};

#endif



