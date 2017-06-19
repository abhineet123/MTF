#ifndef MTF_STRUCK_H
#define MTF_STRUCK_H


#include "Tracker.h"
#include "Config.h"

#include "opencv2/core/core.hpp"

#include "mtf/TrackerBase.h"

#define STRUCK_CONFIG_PATH "Config/struck.cfg"
namespace struck{
	struct StruckParams{
		std::string config_path;
		StruckParams(std::string _config_path);
		StruckParams(const StruckParams *params = nullptr);
	};


	class Struck : public mtf::TrackerBase{
	public:
		typedef StruckParams ParamType;
		Struck(const ParamType *struck_params = nullptr);
		void setImage(const cv::Mat &img) override;
		int inputType() const  override{ return CV_8UC1; }
		void initialize(const cv::Mat& corners) override;
		void update() override;
	private:
		ParamType params;
		Config conf;
		Tracker tracker;
		cv::Mat curr_img;
		cv::Mat curr_img_resized;
		float scaleW, scaleH;
		void updateCVCorners();
	};
}

#endif 
