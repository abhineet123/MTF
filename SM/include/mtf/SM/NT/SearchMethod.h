#ifndef MTF_SEARCH_METHOD_NT_H
#define MTF_SEARCH_METHOD_NT_H

#include "mtf/TrackerBase.h"
#include "mtf/AM/AppearanceModel.h"
#include "mtf/SSM/StateSpaceModel.h"

#include "mtf/Macros/common.h"
#include <memory>

_MTF_BEGIN_NAMESPACE
namespace nt{
	//! Base class for non templated implementations of search methods
	class SearchMethod : public TrackerBase{
	public:
		typedef std::shared_ptr<mtf::AppearanceModel> AM;
		typedef std::shared_ptr<mtf::StateSpaceModel> SSM;

		using TrackerBase::initialize;
		using TrackerBase::update;

		SearchMethod(AM _am, SSM _ssm) :
			TrackerBase(), am(_am), ssm(_ssm),
			spi_mask(nullptr){
			cv_corners_mat.create(2, 4, CV_64FC1);
		}
		SearchMethod() : TrackerBase(),
			am(nullptr), ssm(nullptr),
			spi_mask(nullptr){
			cv_corners_mat.create(2, 4, CV_64FC1);
		}

		virtual ~SearchMethod(){}
		void setImage(const cv::Mat &img) override{
			am->setCurrImg(img);
		}
		const cv::Mat& getRegion() override{
			return cv_corners_mat;
		}
		// default implementation for SMs where the AM processing in the current frame
		// does not depend on the results obtained in the last frame
		void setRegion(const cv::Mat& corners) override{
			ssm->setCorners(corners);
			ssm->getCorners(cv_corners_mat);
		}
		virtual void setSPIMask(const bool *_spi_mask){
			spi_mask = _spi_mask;
			am->setSPIMask(_spi_mask);
			ssm->setSPIMask(_spi_mask);
		}
		virtual void clearSPIMask(){
			spi_mask = nullptr;
			am->clearSPIMask();
			ssm->clearSPIMask();
		}
		virtual const bool* getSPIMask(){
			return spi_mask;
		}

		virtual void setInitStatus(){
			am->setInitStatus();
			ssm->setInitStatus();
		}
		virtual void clearInitStatus(){
			am->clearInitStatus();
			ssm->clearInitStatus();
		}

		virtual bool supportsSPI(){ return am->supportsSPI() && ssm->supportsSPI(); }
		virtual int inputType() const override{ return am->inputType(); }

		//! direct access to the underlying AM and SSM
		virtual AM getAM() const { return am; }
		virtual SSM getSSM() const { return ssm; }

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	protected:
		AM am;
		SSM ssm;
		const bool *spi_mask;
	};
}

_MTF_END_NAMESPACE

#endif
