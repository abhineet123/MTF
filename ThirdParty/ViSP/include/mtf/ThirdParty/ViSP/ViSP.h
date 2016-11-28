#ifndef MTF_VISP_H
#define MTF_VISP_H

#include "mtf/TrackerBase.h"
#include <visp3/tt/vpTemplateTrackerWarp.h>
#include <visp3/tt/vpTemplateTracker.h>
#include <memory>

struct ViSPParams{
	enum class SSMType{
		Homography, SL3, Affine,
		Similarity, Isometry, Translation
	};
	enum class SMType{
		FCLK, ICLK, FALK, ESM
	};
	enum class AMType{
		SSD, ZNCC, MI
	};

	SMType sm_type;
	AMType am_type;
	SSMType ssm_type;

	int max_iters;
	int resx, resy;
	double lambda;
	double thresh_grad;
	int pyr_n_levels;
	int pyr_level_to_stop;

	ViSPParams(SMType _sm_type, AMType _am_type,
		SSMType _ssm_type,
		int _max_iters,
		int _resx,
		int _resy,
		double _lambda,
		double _thresh_grad,
		int _pyr_n_levels,
		int _pyr_level_to_stop);

	ViSPParams(const ViSPParams *params = nullptr);
};

class ViSP : public mtf::TrackerBase{
public:

	typedef ViSPParams ParamType;
	typedef ParamType::SMType SMType;
	typedef ParamType::AMType AMType;
	typedef ParamType::SSMType SSMType;
	typedef std::unique_ptr<vpTemplateTracker> VPTrackerPtr;
	typedef std::unique_ptr<vpTemplateTrackerWarp> VPWarpPtr;

	ViSP(const ParamType *visp_params);	
	void initialize(const cv::Mat &corners) override;
	void update() override;
	const cv::Mat& getRegion() override{ return cv_corners_mat; }
	void setRegion(const cv::Mat& corners) override;
	void setImage(const cv::Mat &img) override;
	int inputType() const override{ return CV_8UC1; }
	void setOptimalSamplingRatio(const cv::Mat &corners);
	void updateCorners();

protected:
	ParamType params;
	cv::Mat curr_img_cv;
	VPWarpPtr warp;
	VPTrackerPtr tracker;
	
private:
	virtual ~ViSP(){}
};
#endif
