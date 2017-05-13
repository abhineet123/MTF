#ifndef MTF_INPUT_CV
#define MTF_INPUT_CV

#include "inputBase.h"
#include "opencv2/highgui/highgui.hpp"


class InputCV : public InputBase {

public:
	InputCV(char img_source = 'u', string _dev_name = "", string _dev_fmt = "",
		string _dev_path = "", int _n_buffers = 1, bool _invert_seq = false,
		int _img_type = CV_8UC3) :
		InputBase(img_source, _dev_name, _dev_fmt, _dev_path, _n_buffers, _invert_seq),
		img_type(_img_type), frame_id(0){}
	~InputCV(){
		cv_buffer.clear();
		cap_obj.release();
	}
	bool initialize() override{
		printf("Initializing OpenCV pipeline...\n");

		n_channels = 3;
		if(img_source == SRC_VID || img_source == SRC_IMG) {
			if(img_source == SRC_VID){
				printf("Opening %s video file: %s\n", dev_fmt.c_str(), file_path.c_str());
				//n_frames = cap_obj.get(CV_CAP_PROP_FRAME_COUNT);
			} else{
				printf("Opening %s image files at %s", dev_fmt.c_str(), file_path.c_str());
				if(n_frames > 0){
					printf(" with %d frames\n", n_frames);
				}
				printf("\n");
			}
			cap_obj.open(file_path);
		} else if(img_source == SRC_USB_CAM) {
			if(dev_path.empty()){
				cap_obj.open(0);
			} else{
				cap_obj.open(atoi(dev_path.c_str()));
			}
		} else {
			printf("Invalid source provided for OpenCV Pipeline: %c\n", img_source);
			return false;
		}
		if(!(cap_obj.isOpened())) {
			printf("OpenCV pipeline could not be initialized successfully\n");
			return false;
		} 
			
		//Mat temp_frame;
		//*cap_obj>>temp_frame;
		//frames_captured++;

		//img_height=temp_frame.rows;
		//img_width=temp_frame.cols;

		img_height = static_cast<int>(cap_obj.get(CV_CAP_PROP_FRAME_HEIGHT));
		img_width = static_cast<int>(cap_obj.get(CV_CAP_PROP_FRAME_WIDTH));

		/*img_height=cap_obj.get(CV_CAP_PROP_FRAME_HEIGHT);
		img_width=cap_obj.get(CV_CAP_PROP_FRAME_WIDTH);*/
		printf("OpenCV pipeline initialized successfully to grab frames of size: %d x %d\n", 
			img_width, img_height);
		cv_buffer.resize(n_buffers);
		for(int i = 0; i < n_buffers; ++i){
			cv_buffer[i].create(img_height, img_width, img_type);
		}
		buffer_id = 0;
		frame_id = 0;
		if(invert_seq){
			printf("Reading sequence into buffer....\n");
			for(int i = 0; i < n_buffers; ++i){
				if(!cap_obj.read(cv_buffer[n_buffers - i - 1])){ return false; };
			}
			return true;
		}
		return cap_obj.read(cv_buffer[buffer_id]);
	}
	bool update() override{
		buffer_id = (buffer_id + 1) % n_buffers;
		++frame_id;
		if(invert_seq){
			return true;
		}
		return cap_obj.read(cv_buffer[buffer_id]);
	}

	void remapBuffer(unsigned char** new_addr) override{
		if(invert_seq){
			for(int i = 0; i < n_buffers; ++i){
				cv_buffer[i].copyTo(cv::Mat(cv_buffer[i].rows, cv_buffer[i].cols, cv_buffer[i].type(), new_addr[i]));
				cv_buffer[i].data = new_addr[i];
			}
		} else{
			for(int i = 0; i < n_buffers; ++i){
				//printf("Remapping CV buffer %d to: %lu\n", i, (unsigned long)new_addr[i]);
				cv_buffer[i].data = new_addr[i];
			}
			buffer_id = -1;
			update();
		}
	}
	const cv::Mat& getFrame() const override{
		return cv_buffer[buffer_id];
	}
	cv::Mat& getFrame(FrameType frame_type) override{
		return cv_buffer[buffer_id];
	}
	int getFrameID() const override{ return frame_id; }

private:

	cv::VideoCapture cap_obj;
	vector<cv::Mat> cv_buffer;
	int img_type;
	int frame_id;
};

#endif



