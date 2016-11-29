#ifndef MTF_INPUT_BASE
#define MTF_INPUT_BASE

#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "mtf/Config/parameters.h"
#include "boost/filesystem/operations.hpp"

#ifdef _WIN32
#define snprintf  _snprintf
#endif

#define NCHANNELS 3

#define SRC_VID 'm'
#define SRC_USB_CAM 'u'
#define SRC_FW_CAM 'f'
#define SRC_IMG 'j'
#define SRC_DISK 'd'

#define MAX_PATH_SIZE 500

using namespace std;
namespace fs = boost::filesystem;

enum FrameType{ MUTABLE };

class InputBase {
protected:
	int img_width;
	int img_height;
	int n_buffers;
	int n_channels;
	int buffer_id;


	char img_source;
	string file_path;
	string dev_name;
	string dev_fmt;
	string dev_path;

public:
	int n_frames;

	InputBase() : img_width(0), img_height(0), n_buffers(0),
		n_channels(0), buffer_id(0), img_source('j'), n_frames(0){}
	InputBase(char _img_source, string _dev_name, string _dev_fmt,
		string _dev_path, int _n_buffers = 1) :
		n_buffers(_n_buffers),	img_source(_img_source),dev_name(_dev_name),
		dev_fmt(_dev_fmt),	dev_path(_dev_path),n_frames(0){

		printf("InputBase :: img_source: %c\n", img_source);

		if(img_source == SRC_VID){
			if(dev_fmt.empty()){ dev_fmt = "mpg"; }
			if(dev_path.empty()){ dev_path = "."; }
			file_path = dev_path + "/" + dev_name + "." + dev_fmt;
			if(!fs::exists(file_path)){
				throw std::invalid_argument(
					cv::format("InputBase :: Video file %s does not exist", file_path.c_str()));
			}
			n_frames = getNumberOfVideoFrames(file_path.c_str());
		} else if(img_source == SRC_IMG || img_source == SRC_DISK){
			if(dev_fmt.empty()){ dev_fmt = "jpg"; }
			if(dev_path.empty()){ dev_path = "."; }
			std::string img_folder_path = dev_path + "/" + dev_name;
			if(!fs::exists(img_folder_path)){
				throw std::invalid_argument(
					cv::format("InputBase :: Image sequence folder %s does not exist", img_folder_path.c_str()));
			}

			file_path = img_folder_path + "/frame%05d." + dev_fmt;

			n_frames = getNumberOfFrames(file_path.c_str());
		}
		printf("dev_name: %s\n", dev_name.c_str());
		printf("dev_path: %s\n", dev_path.c_str());
		printf("dev_fmt: %s\n", dev_fmt.c_str());
	}

	virtual ~InputBase(){}
	virtual bool initialize() = 0;
	virtual bool update() = 0;

	virtual void remapBuffer(unsigned char **new_addr) = 0;
	virtual int getFrameID() const = 0;
	virtual const cv::Mat& getFrame() const = 0;
	//! apparently a const function cannot return a non const object by reference
	virtual cv::Mat& getFrame(FrameType frame_type) = 0;
	virtual int getHeight() const{ return img_height; }
	virtual int getWidth() const{ return img_width; }


	int getNumberOfFrames(const char *file_template){
		int frame_id = 0;
		while(FILE* fid = fopen(cv::format(file_template, ++frame_id).c_str(), "r")){
			fclose(fid);
		}
		return frame_id - 1;
	}

	int getNumberOfVideoFrames(const char *file_name){
		cv::VideoCapture cap_obj(file_name);
		cv::Mat img;
		int frame_id = 0;
		while(cap_obj.read(img)){ ++frame_id; }
		cap_obj.release();
		return frame_id;
	}
};
#endif
