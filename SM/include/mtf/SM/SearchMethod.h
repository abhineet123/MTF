#ifndef MTF_SEARCH_METHOD_H
#define MTF_SEARCH_METHOD_H

#include "mtf/TrackerBase.h"
#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE

template<class AM, class SSM>
class SearchMethod : public TrackerBase{
protected:
	AM am;
	SSM ssm;

public:
	using TrackerBase::initialize;
	using TrackerBase::update;

	typedef typename  AM::ParamType AMParams;
	typedef typename SSM::ParamType SSMParams;

	SearchMethod(const AMParams *am_params, const SSMParams *ssm_params) :
		TrackerBase(), am(am_params), ssm(ssm_params),
		spi_mask(nullptr){
		cv_corners_mat.create(2, 4, CV_64FC1);
	}
	SearchMethod() : TrackerBase(),
		spi_mask(nullptr){
		cv_corners_mat.create(2, 4, CV_64FC1);
	}

	virtual ~SearchMethod(){}
	void setImage(const cv::Mat &img) override{
		am.setCurrImg(img);
	}
	const cv::Mat& getRegion() override{
		return cv_corners_mat;
	}
	// default implementation for SMs where the AM processing in the current frame
	// does not depend on the results obtained in the last frame
	void setRegion(const cv::Mat& corners) override{
		ssm.setCorners(corners);
		ssm.getCorners(cv_corners_mat);
	}

	const bool *spi_mask;
	virtual void setSPIMask(const bool *_spi_mask){
		spi_mask = _spi_mask;
		am.setSPIMask(_spi_mask);
		ssm.setSPIMask(_spi_mask);
	}
	virtual void clearSPIMask(){
		spi_mask = nullptr;
		am.clearSPIMask();
		ssm.clearSPIMask();
	}

	virtual void setInitStatus(){
		am.setInitStatus();
		ssm.setInitStatus();
	}
	virtual void clearInitStatus(){
		am.clearInitStatus();
		ssm.clearInitStatus();
	}

	virtual bool supportsSPI(){ return am.supportsSPI() && ssm.supportsSPI(); }

	virtual int inputType() const override{ return am.inputType(); }

	//! direct access to the underlying AM and SSM
	virtual AM& getAM() { return am; }
	virtual SSM& getSSM() { return ssm; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// partial specialization for a SM with a 
// composite or implicit appearance model that cannot be expressed as a class
template <class SSM>
class SearchMethod<void, SSM> : public TrackerBase{
protected:

	SSM ssm;

public:

	typedef typename SSM::ParamType SSMParams;

	SearchMethod(const SSMParams *ssm_params) :
		TrackerBase(), ssm(ssm_params),
		spi_mask(nullptr){
		cv_corners_mat.create(2, 4, CV_64FC1);
	}
	virtual ~SearchMethod(){}

	const cv::Mat& getRegion() override{
		return cv_corners_mat;
	}
	// default implementation for SMS where the AM processing in the current frame
	// does not depend on the results obtained in the last frame
	void setRegion(const cv::Mat& corners) override{
		ssm.setCorners(corners);
		ssm.getCorners(cv_corners_mat);
	}

	const bool *spi_mask;
	virtual void setSPIMask(const bool *_spi_mask){
		spi_mask = _spi_mask;
		ssm.setSPIMask(_spi_mask);
	}
	virtual void clearSPIMask(){
		spi_mask = nullptr;
		ssm.clearSPIMask();
	}
	virtual bool supportsSPI(){ return ssm.supportsSPI(); }
	virtual SSM& getSSM() { return ssm; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

_MTF_END_NAMESPACE

#endif
