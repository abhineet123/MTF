#ifndef MTF_INPUT_CV
#define MTF_INPUT_CV

#include "inputBase.h"
#include "opencv2/highgui/highgui.hpp"


class InputCV : public InputBase {

public:

	cv::VideoCapture cap_obj;
	vector<cv::Mat> cv_buffer;

	int img_type;
	int frame_id;

	InputCV(char img_source = 'u', string _dev_name = "", string _dev_fmt = "",
		string _dev_path = "", int _n_buffers = 1, int _img_type = CV_8UC3) :
		InputBase(img_source, _dev_name, _dev_fmt, _dev_path, _n_buffers),
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
			printf("OpenCV stream could not be initialized successfully\n");
			return false;
		} else{
			printf("OpenCV stream initialized successfully\n");
		}

		//Mat temp_frame;
		//*cap_obj>>temp_frame;
		//frames_captured++;

		//img_height=temp_frame.rows;
		//img_width=temp_frame.cols;

		img_height = cap_obj.get(CV_CAP_PROP_FRAME_HEIGHT);
		img_width = cap_obj.get(CV_CAP_PROP_FRAME_WIDTH);

		/*img_height=cap_obj.get(CV_CAP_PROP_FRAME_HEIGHT);
		img_width=cap_obj.get(CV_CAP_PROP_FRAME_WIDTH);*/

		printf("InputCV :: Images are of size: %d x %d\n", img_width, img_height);

		cv_buffer.resize(n_buffers);

		for(int i = 0; i < n_buffers; i++){
			cv_buffer[i].create(img_height, img_width, img_type);
		}
		buffer_id = -1;

		frame_id = 0;
		buffer_id = (buffer_id + 1) % n_buffers;
		return cap_obj.read(cv_buffer[buffer_id]);
	}
	bool update() override{
		buffer_id = (buffer_id + 1) % n_buffers;
		++frame_id;
		return cap_obj.read(cv_buffer[buffer_id]);
	}

	void remapBuffer(uchar** new_addr) override{
		for(int i = 0; i < n_buffers; i++){
			//printf("Remapping CV buffer %d to: %lu\n", i, (unsigned long)new_addr[i]);
			cv_buffer[i].data = new_addr[i];
		}
		buffer_id = -1;
		update();
	}
	const cv::Mat& getFrame() const override{
		return cv_buffer[buffer_id];
	}
	cv::Mat& getFrame(FrameType frame_type) override{
		return cv_buffer[buffer_id];
	}
	int getFrameID() const override{ return frame_id; }
};

#endif



