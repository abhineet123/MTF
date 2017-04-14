#ifndef MTF_GOTURN_H
#define MTF_GOTURN_H

#include "network/regressor.h"
#include "tracker/tracker.h"

#include "mtf/TrackerBase.h"

#define GOTURN_DO_TRAIN true
#define GOTURN_GPU_ID 0
#define GOTURN_SHOW_INTERMEDIATE_OUTPUT false
#define GOTURN_MODEL_FILE "Data/GOTURN/tracker.prototxt"
#define GOTURN_TRAINED_FILE "Data/GOTURN/solver.prototxt"


struct GOTURNParams{
	bool do_train;
	int gpu_id;
	bool show_intermediate_output;
	std::string model_file;
	std::string trained_file;
	GOTURNParams(bool _do_train, int _gpu_id,
		bool _show_intermediate_output, const char* model_file,
		const char* trained_file);
	GOTURNParams(const GOTURNParams *params = nullptr);
};


class GOTURN : public mtf::TrackerBase{
public:
	typedef GOTURNParams ParamType;
	GOTURN(const GOTURNParams *gtrn_params = nullptr);
	~GOTURN();

	int inputType() const override{ return CV_8UC3; }
	void initialize(const cv::Mat& cv_corners) override;
	void update() override;
	void setImage(const cv::Mat &img)  override{ curr_img = img; }

private:
	ParamType params;

	Regressor *regressor;
	Tracker *tracker;

	cv::Mat curr_img;
	BoundingBox bbox_estimate;
	std::vector<float> bounding_box_vec;

	void updateCVCorners();
};

#endif /* end of include guard: GOTURN_H */
