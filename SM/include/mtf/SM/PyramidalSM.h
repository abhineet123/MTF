#ifndef MTF_PYRAMIDAL_SM_H
#define MTF_PYRAMIDAL_SM_H

#include "CompositeSM.h"
#include "mtf/SM/PyramidalParams.h"

_MTF_BEGIN_NAMESPACE

/**
run multiple search methods with the same AM/SSM on a Gaussian image pyramid; although the code here
is currently identical to CascadeTracker, a seperate module exists for future extensions
that take advantage of the additional information that is available to the Cascade through
direct access to the underlying AM and SSM
*/
template<class AM, class SSM>
class PyramidalSM : public CompositeSM < AM, SSM > {

public:
	typedef PyramidalParams ParamType;

	using typename CompositeSM<AM, SSM>::SM;
	typedef typename SM::SSMParams SSMParams;
	using CompositeSM<AM, SSM>::name;
	using CompositeSM<AM, SSM>::cv_corners_mat;
	using CompositeSM<AM, SSM>::trackers;
	using CompositeSM<AM, SSM>::n_trackers;
	using CompositeSM<AM, SSM>::input_type;

	PyramidalSM(const vector<SM*> &_trackers, 
		const ParamType *parl_params);
	void setImage(const cv::Mat &img) override;
	void initialize(const cv::Mat &corners) override;
	void update() override;
	const cv::Mat& getRegion() override { 
		return trackers[0]->getRegion();
	}
	void setRegion(const cv::Mat& corners)  override;

protected:

	ParamType params;

	vector<cv::Size> img_sizes;
	vector<cv::Mat> img_pyramid;
	double overall_scale_factor;
	bool external_img_pyramid;

	void setImagePyramid(const vector<cv::Mat> &_img_pyramid);
	const vector<cv::Mat>& getImagePyramid() const{ return img_pyramid; }
	void updateImagePyramid();
	void showImagePyramid();
};

_MTF_END_NAMESPACE

#endif

