#ifndef MTF_INPUT_XV_H
#define MTF_INPUT_XV_H

#include "inputBase.h"
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

#include <XVVideo.h>
#include <XVMpeg.h>
#include <XVAVI.h>
#include <XVV4L2.h>
#include <XVDig1394.h>
#include <XVImageRGB.h>
#include <XVImageSeq.h>

#include <memory>

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
		int n_buffers_in, int n_frames) : vid(nullptr), n_buffers(n_buffers_in){
		switch(img_source) {
		case SRC_IMG: {
			fprintf(stdout, "Opening jpeg file at %s with %d frames\n", file_path.c_str(), n_frames);
			vid.reset(new IMG(file_path.c_str(), 1, n_frames, n_buffers));
			break;
		}
		case SRC_USB_CAM: {
			fprintf(stdout, "Opening USB camera %s with format %s\n", dev_path.c_str(), dev_fmt.c_str());
			vid.reset(new V4L2(dev_path.c_str(), dev_fmt.c_str()));
			break;
		}
		case SRC_FW_CAM: {
			fprintf(stdout, "Opening FIREWIRE camera %s  with format %s\n", dev_path.c_str(), dev_fmt.c_str());
			vid.reset(new DIG1394(dev_path.c_str(), dev_fmt.c_str(), DIG1394_NTH_CAMERA(0)));
			break;
		}
		default: {
			fprintf(stdout, "Invalid image source provided for 24 bit Xvision pipeline\n");
			exit(0);
		}
		}
		XVSize img_size = vid->get_size();
		img_height = img_size.Height();
		img_width = img_size.Width();
	}	
	void initFrame(int init_buffer_id) override{
		if(vid->initiate_acquire(init_buffer_id) < 0){
			cout << "Error in InputXV24: Frame could not be acquired\n";
		}
	}

	void initBuffer(PIX_TYPE24 **init_addrs, 
		IMAGE_TYPE24 *buffer_addrs) override{
		vid_buffers = vid->remap(init_addrs, n_buffers);
		for(int i = 0; i < n_buffers; i++){
			vid_buffers[i].resize(img_width, img_height);
			vid_buffers[i].remap((PIX_TYPE24*)buffer_addrs[i].data(), false);
		}
	}

	void updateFrame(int buffer_id) override{
		if(vid->initiate_acquire((buffer_id + 1) % n_buffers) < 0){
			cout << "Error in InputXV24: Frame could not be acquired from the input stream\n";
			return;
		}
		vid->wait_for_completion(buffer_id);
	}

	void updateBuffer(PIX_TYPE24 **new_addrs) override{
		for(int i = 0; i < n_buffers; i++){
			vid_buffers[i].remap(new_addrs[i], false);
		}
	}

private:

	std::unique_ptr<VID24> vid;
	IMAGE_TYPE24 *vid_buffers;
	int n_buffers;
};

class InputXV32 : public InputXVSource{

public:
	InputXV32(int img_source, string file_path) : vid(nullptr){

		switch(img_source) {
		case SRC_VID: {
			fprintf(stdout, "Opening video file %s\n", file_path.c_str());
			vid.reset(new MPG(file_path.c_str()));
			break;
		}
		default: {
			fprintf(stdout, "Invalid image source provided for 32 bit Xvision pipeline\n");
			exit(0);
		}
		}
		XVSize img_size = vid->get_size();
		img_height = img_size.Height();
		img_width = img_size.Width();

	}

	void initFrame(int init_buffer_id) override{
		buffer_id32 = init_buffer_id;
		if(vid->initiate_acquire(init_buffer_id) < 0){
			cout << "Error in InputXV32:: Frame could not be acquired\n";
			return;
		}
	}

	void initBuffer(PIX_TYPE24 **init_addrs, 
		IMAGE_TYPE24 *buffer_addrs) override{
		buffer24 = buffer_addrs;
	}

	// XVMpeg does not support 24 bit images so it has to be used to 
	// capture 32 bit images and then copy data to the 24 bit buffers
	void updateFrame(int buffer_id24) override{
		// XVMpeg does not provide any option to set the number of buffers so we have to use 
		// the default hard coded there in XVMpeg.h as 'DEF_MPEG_NUM'
		buffer_id32 = (buffer_id32 + 1) % DEF_MPEG_NUM;
		//printf("**************************getFrameMPG**********************\n");
		if(vid->initiate_acquire(buffer_id32) < 0){
			cout << "Error in InputXV32: Frame could not be acquired from the input stream\n";
			return;
		}
		vid->wait_for_completion(buffer_id32);
		copyFrame32ToBuffer24(vid->frame(buffer_id32), buffer_id24);
	}

	void updateBuffer(PIX_TYPE24 **new_addrs) override{}

	// copy image data from 32 bit source image to 24 bit buffer image
	void copyFrame32ToBuffer24(IMAGE_TYPE32 &src_frame, int buffer_id24) {
		unsigned char* src_data = (unsigned char*)(src_frame.data());
		unsigned char* dst_data = (unsigned char*)(buffer24[buffer_id24].data());

		for(int row = 0; row < img_height; row++) {
			for(int col = 0; col < img_width; col++) {
				memcpy(dst_data, src_data, sizeof(PIX_TYPE24));
				dst_data += sizeof(PIX_TYPE24);
				src_data += sizeof(PIX_TYPE32);
			}
		}
	}

private:

	std::unique_ptr<VID32> vid;
	IMAGE_TYPE24 *buffer24;

	int buffer_id32;
};

class InputXV : public InputBase {

public:
	typedef float PIX_TYPE_GS;
	typedef XV_RGB24 PIX_TYPE;
	typedef XVImageRGB<PIX_TYPE> IMAGE_TYPE;
	typedef XVImageScalar<PIX_TYPE_GS> IMAGE_TYPE_GS;

	InputXV(char img_source, string _dev_name, string _dev_fmt,
		string _dev_path, int _n_buffers = 1, bool _invert_seq = false) :
		InputBase(img_source, _dev_name, _dev_fmt, _dev_path, _n_buffers, _invert_seq),
		src(nullptr){}

	~InputXV(){
		cv_buffer.clear();
		xv_buffer.clear();
	}

	bool initialize() override{
		if(invert_seq && (img_source == SRC_FW_CAM || img_source == SRC_USB_CAM)){
			printf("InputXV :: Inverted sequence cannot be used with live input");
			return false;
		}

		n_channels = sizeof(PIX_TYPE);

		if(img_source == SRC_VID){
			src.reset(new InputXV32(img_source, file_path));
		} else{
			src.reset(new InputXV24(img_source, dev_path, dev_fmt, file_path, n_buffers, n_frames));
		}

		img_height = src->img_height;
		img_width = src->img_width;

		if(img_height == 0 || img_width == 0){
			return false;
		}

		printf("Xvision pipeline initialized successfully to grab frames of size: %d x %d\n",
			img_width, img_height);

		vector<PIX_TYPE*> data_addrs;
		cv_buffer.resize(n_buffers);
		xv_buffer.resize(n_buffers);
		for(int i = 0; i < n_buffers; ++i){
			cv_buffer[i].create(img_height, img_width, CV_8UC3);
			xv_buffer[i] = IMAGE_TYPE(img_width, img_height);
			data_addrs.push_back((PIX_TYPE*)(xv_buffer[i].data()));
			cv_buffer[i].data = (unsigned char*)xv_buffer[i].data();
		}
		src->initBuffer(data_addrs.data(), xv_buffer.data());
		buffer_id = 0;
		frame_id = 0;
		if(invert_seq){
			printf("Reading sequence into buffer....\n");
			for(int i = 0; i < n_buffers; ++i){
				src->updateFrame(n_buffers - i - 1);
			}
			return true;
		}		
		src->updateFrame(buffer_id);		
		return true;
	}

	bool update() override{
		++frame_id;
		buffer_id = (buffer_id + 1) % n_buffers;
		if(!invert_seq){
			src->updateFrame(buffer_id);
		}		
		return true;
	}

	void remapBuffer(unsigned char** new_addr) override{
		//vid->own_buffers=0;
		//vid_buffers=vid->remap((PIX_TYPE**)new_addr, n_buffers);
		src->updateBuffer((PIX_TYPE**)new_addr);
		for(int i = 0; i < n_buffers; ++i){
			//vid_buffers[i].resize(img_width, img_height);			
			/*IMAGE_TYPE* temp_frame=&(vid->frame(i));

			//temp_frame->pixmap->own_flag=false;
			temp_frame->resize(img_width, img_height);
			temp_frame->remap((PIX_TYPE*)new_addr[i], false);*/
			xv_buffer[i].remap((PIX_TYPE*)(new_addr[i], false));
			cv_buffer[i].data = new_addr[i];
			//temp_frame->pixmap=new XVPixmap<PIX_TYPE>(img_width, img_height, (PIX_TYPE*)new_addr[i], false);
			//temp_frame->win_addr=temp_frame->pixmap->buffer_addr;
		}
		//vid->remap((PIX_TYPE**)new_addr, n_buffers);
		buffer_id = -1;
		src->initFrame(0);
		update();
	}

	const cv::Mat& getFrame() const override{
		return cv_buffer[buffer_id];
	}
	cv::Mat& getFrame(FrameType frame_type) override{
		return cv_buffer[buffer_id];
	}
	int getFrameID() const override{ return frame_id; }

private:

	std::unique_ptr<InputXVSource> src;
	vector<IMAGE_TYPE> xv_buffer;
	vector<cv::Mat> cv_buffer;
	int frame_id;

	void copyXVToCV(IMAGE_TYPE_GS &xv_img, cv::Mat &cv_img) {
		//printf("Copying XV to CV\n");
		int img_height = xv_img.SizeY();
		int img_width = xv_img.SizeX();
		//printf("img_height: %d, img_width=%d\n", img_height, img_width);
		/*cv_img=new cv::Mat(img_height, img_width, CV_8UC1);
		printf("created image with img_height: %d, img_width: %d\n",
		cv_img->rows, cv_img->cols);*/
		PIX_TYPE_GS* xv_data = (PIX_TYPE_GS*)xv_img.data();
		unsigned char* cv_data = cv_img.data;
		for(int row = 0; row < img_height; row++) {
			//printf("row: %d\n", row);
			for(int col = 0; col < img_width; col++) {
				//printf("\tcol: %d\n", col);
				int xv_location = col + row * img_width;
				int cv_location = (col + row * img_width);
				cv_data[cv_location] = (unsigned char)xv_data[xv_location];
			}
		}
		//printf("done\n");
	}

	void copyXVToCVIter(IMAGE_TYPE_GS &xv_img, cv::Mat &cv_img) {
		int img_width = xv_img.Width();
		int img_height = xv_img.Height();
		//printf("img_height: %d, img_width=%d\n", img_height, img_width);
		int cv_location = 0;
		XVImageIterator<PIX_TYPE_GS> iter(xv_img);
		unsigned char* cv_data = cv_img.data;
		for(; !iter.end(); ++iter){
			cv_data[cv_location++] = *iter;
		}
	}
	void copyXVToCVIter(IMAGE_TYPE &xv_img, cv::Mat &cv_img) {
		int img_width = xv_img.Width();
		int img_height = xv_img.Height();
		//printf("img_height: %d, img_width=%d\n", img_height, img_width);
		int cv_location = 0;
		XVImageIterator<PIX_TYPE> iter(xv_img);
		unsigned char* cv_data = cv_img.data;
		for(; !iter.end(); ++iter){
			cv_data[cv_location++] = iter->b;
			cv_data[cv_location++] = iter->g;
			cv_data[cv_location++] = iter->r;
		}
	}
};

#endif

