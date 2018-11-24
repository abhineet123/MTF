#ifndef MTF_H
#define MTF_H

//! External interface of the MTF library; this is the only header that needs to be included to use the framework
//! Provides functions to create trackers corresponding to different combinations of 
//! search methods, appearance models and state space models as well as some third party trackers

#ifdef _WIN32
#pragma warning(disable:4800)
#endif

//! parameters for the different modules
#include "mtf/Config/parameters.h"

//! search methods
#ifndef DISABLE_TEMPLATED_SM
//! templated implementations
#include "mtf/SM/ESM.h"
#include "mtf/SM/ICLK.h"
#include "mtf/SM/FCLK.h"
#include "mtf/SM/FALK.h"
#include "mtf/SM/IALK.h"
#include "mtf/SM/PF.h"
//! composite search methods
#include "mtf/SM/CascadeSM.h"
#include "mtf/SM/ParallelSM.h"
#include "mtf/SM/PyramidalSM.h"
#endif
#ifndef DISABLE_FLANN
#include "mtf/SM/NN.h"
#endif
#ifndef DISABLE_GRID
#include "mtf/SM/GridTracker.h"
#include "mtf/SM/GridTrackerCV.h"
#ifndef DISABLE_TEMPLATED_SM
#include "mtf/SM/RKLT.h"
#include "mtf/SM/GridTrackerFlow.h"
#endif
#endif
#ifndef DISABLE_FEAT
#include "mtf/SM/FeatureTracker.h"
#endif
#include "mtf/SM/CascadeTracker.h"
#include "mtf/SM/ParallelTracker.h"
#include "mtf/SM/PyramidalTracker.h"
#include "mtf/SM/LineTracker.h"
//! Non templated implementations
#include "mtf/SM/NT/FCLK.h"
#include "mtf/SM/NT/ICLK.h"
#include "mtf/SM/NT/FALK.h"
#include "mtf/SM/NT/IALK.h"
#include "mtf/SM/NT/ESM.h"
#include "mtf/SM/NT/AESM.h"
#include "mtf/SM/NT/PF.h"
#include "mtf/SM/NT/NN.h"
#include "mtf/SM/NT/FCSD.h"
#include "mtf/SM/NT/GridTrackerFlow.h"
#ifndef DISABLE_REGNET
#include "mtf/SM/NT/RegNet.h"
#endif
#include "mtf/SM/NT/RKLT.h"

//! appearance models
#include "mtf/AM/SSD.h"
#include "mtf/AM/NSSD.h"
#include "mtf/AM/ZNCC.h"
#include "mtf/AM/SCV.h"
#include "mtf/AM/LSCV.h"
#include "mtf/AM/RSCV.h"
#include "mtf/AM/LRSCV.h"	
#include "mtf/AM/MI.h"
#include "mtf/AM/CCRE.h"
#include "mtf/AM/SPSS.h"
#include "mtf/AM/SSIM.h"
#include "mtf/AM/NCC.h"
#include "mtf/AM/RIU.h"
#include "mtf/AM/NGF.h"
#include "mtf/AM/SAD.h"
#include "mtf/AM/KLD.h"
#include "mtf/AM/LKLD.h"
#ifndef DISABLE_DFM
#include "mtf/AM/DFM.h"
#endif
#ifndef DISABLE_PCA
#include "mtf/AM/PCA.h"
#endif
//! multi channel variants
#include "mtf/AM/MCSSD.h"
#include "mtf/AM/MCSCV.h"
#include "mtf/AM/MCLSCV.h"
#include "mtf/AM/MCRSCV.h"
#include "mtf/AM/MCZNCC.h"
#include "mtf/AM/MCNCC.h"
#include "mtf/AM/MCMI.h"
#include "mtf/AM/MCSSIM.h"
#include "mtf/AM/MCSPSS.h"
#include "mtf/AM/MCCCRE.h"
#include "mtf/AM/MCRIU.h"
#include "mtf/AM/MCSAD.h"
#ifndef DISABLE_PCA
#include "mtf/AM/MCPCA.h"
#endif
//! composite AMs
#include "mtf/AM/SumOfAMs.h"
//! illumination models
#include "mtf/AM/GB.h"
#include "mtf/AM/PGB.h"
#include "mtf/AM/RBF.h"

//! state space models
#include "mtf/SSM/Homography.h"
#include "mtf/SSM/LieHomography.h"
#include "mtf/SSM/CBH.h"
#include "mtf/SSM/SL3.h"
#include "mtf/SSM/Affine.h"
#include "mtf/SSM/LieAffine.h"
#include "mtf/SSM/ASRT.h"
#include "mtf/SSM/Similitude.h"
#include "mtf/SSM/Isometry.h"
#include "mtf/SSM/AST.h"
#include "mtf/SSM/IST.h"
#include "mtf/SSM/Translation.h"
#include "mtf/SSM/Spline.h"

//! Third Party Trackers
#ifndef DISABLE_THIRD_PARTY_TRACKERS
#include "mtf/ThirdParty/DSST/DSST.h"
#include "mtf/ThirdParty/KCF/KCF.h"
#include "mtf/ThirdParty/RCT/RCT.h"
#include "mtf/ThirdParty/Struck/Struck.h"
#include "mtf/ThirdParty/CMT/CMT.h"
#include "mtf/ThirdParty/FRG/FRG.h"
#ifndef DISABLE_TLD
#include "mtf/ThirdParty/TLD/TLD.h"
#endif
#ifndef DISABLE_MIL
#include "mtf/ThirdParty/MIL/MIL.h"
#endif
#ifndef DISABLE_PFSL3
#include "mtf/ThirdParty/PFSL3/PFSL3.h"
#endif
#ifndef DISABLE_VISP
#include "mtf/ThirdParty/ViSP/ViSP.h"
#endif
#ifndef DISABLE_CV3
#include "mtf/ThirdParty/CV3/CV3.h"
#endif
//! some thirdparty trackers do not compile in Windows yet
#ifndef DISABLE_DFT
#include "mtf/ThirdParty/DFT/DFT.h"
#endif
#ifndef DISABLE_GOTURN
#include "mtf/ThirdParty/GOTURN/GOTURN.h"
#endif
#ifndef DISABLE_XVISION
//! Xvision trackers
#include "mtf/ThirdParty/Xvision/xvSSDTrans.h"
#include "mtf/ThirdParty/Xvision/xvSSDAffine.h"
#include "mtf/ThirdParty/Xvision/xvSSDSE2.h"
#include "mtf/ThirdParty/Xvision/xvSSDRT.h"
#include "mtf/ThirdParty/Xvision/xvSSDRotate.h"
#include "mtf/ThirdParty/Xvision/xvSSDScaling.h"
#include "mtf/ThirdParty/Xvision/xvSSDPyramidTrans.h"
#include "mtf/ThirdParty/Xvision/xvSSDPyramidAffine.h"
#include "mtf/ThirdParty/Xvision/xvSSDPyramidSE2.h"
#include "mtf/ThirdParty/Xvision/xvSSDPyramidRT.h"
#include "mtf/ThirdParty/Xvision/xvSSDPyramidRotate.h"
#include "mtf/ThirdParty/Xvision/xvColor.h"
#include "mtf/ThirdParty/Xvision/xvEdgeTracker.h"
#include "mtf/ThirdParty/Xvision/xvSSDGrid.h"
#include "mtf/ThirdParty/Xvision/xvSSDGridLine.h"
bool using_xv_tracker = false;
#endif
#endif

#ifdef HEADER_ONLY_MODE
/**
treat source files as headers so no linking is needed
*/
#include "mtf_src.h"
#endif

#include "boost/filesystem/operations.hpp"

#include <memory>

#define cast_params(Module) static_cast<Module::ParamType*>(params.get())
#define get_sm_params(SMType) getSMParams<SMType::ParamType>().get()

_MTF_BEGIN_NAMESPACE

using namespace params;

template< class AMType, class SSMType >
TrackerBase *getTracker(const char *sm_type,
	const typename AMType::ParamType *am_params = nullptr,
	const typename SSMType::ParamType *ssm_params = nullptr);
TrackerBase *getTracker(const char *sm_type,
	const char *am_type, const char *ssm_type, const char *ilm_type);
//! 3rd party trackers
TrackerBase *getTracker(const char *sm_type);
nt::SearchMethod *getSM(const char *sm_type, const char *am_type,
	const char *ssm_type, const char *ilm_type);
TrackerBase *getCompositeSM(const char *sm_type,
	const char *am_type, const char *ssm_type, const char *ilm_type);
IlluminationModel *getILM(const char *ilm_type);

typedef std::shared_ptr<AppearanceModel> AM;
typedef std::shared_ptr<StateSpaceModel> SSM;

typedef std::unique_ptr<AMParams> AMParams_;
typedef std::unique_ptr<SSMParams> SSMParams_;
typedef std::unique_ptr<SSMEstimatorParams> SSMEstParams_;

typedef utils::spi::ParamsType SPIParamsType;
typedef utils::spi::Types SPIType;

typedef std::unique_ptr<ESMParams> ESMParams_;
typedef std::unique_ptr<FCLKParams> FCLKParams_;
typedef std::unique_ptr<ICLKParams> ICLKParams_;
typedef std::unique_ptr<FALKParams> FALKParams_;
typedef std::unique_ptr<IALKParams> IALKParams_;
typedef std::unique_ptr<FCSDParams> FCSDParams_;
typedef std::unique_ptr<PFParams> PFParams_;
typedef std::unique_ptr<NNParams> NNParams_;

#ifndef DISABLE_FLANN
typedef std::unique_ptr<FLANNParams> FLANNParams_;
#endif

#ifndef DISABLE_FEAT
typedef std::unique_ptr<FLANNCVParams> FLANNCVParams_;
typedef FeatureTrackerParams::DetectorType DetectorType;
typedef FeatureTrackerParams::DescriptorType DescriptorType;
typedef FeatureTrackerParams::DetectorParamsType DetectorParamsType;
typedef FeatureTrackerParams::DescriptorParamsType DescriptorParamsType;

FLANNCVParams_ getFLANNCVParams();
DetectorParamsType getDetectorParams(DetectorType detector_type);
DescriptorParamsType getDescriptorParams(DescriptorType descriptor_type);
#endif

#ifndef DISABLE_REGNET
typedef std::unique_ptr<RegNetParams> RegNetParams_;
#endif

AMParams_ getAMParams(const char *am_type, const char *ilm_type);
SSMParams_ getSSMParams(const char *ssm_type);
SSMEstParams_ getSSMEstParams();
SPIParamsType getSPIParams();
ESMParams_ getESMParams();
FCLKParams_ getFCLKParams();
ICLKParams_ getICLKParams();
FALKParams_ getFALKParams();
IALKParams_ getIALKParams();
PFParams_ getPFParams();
NNParams_ getNNParams();
#ifndef DISABLE_FLANN
FLANNParams_ getFLANNParams();
#endif
#ifndef DISABLE_REGNET
RegNetParams_ getRegNetParams();
#endif
ImageBase *getPixMapper(const char *pix_mapper_type);

//! Multi layer Particle Filter
template< class AMType, class SSMType >
bool getPFk(vector<SearchMethod<AMType, SSMType>*> &trackers,
	const typename AMType::ParamType *am_params = nullptr,
	const typename SSMType::ParamType *ssm_params = nullptr){
	int n_pfk_ssm_sigma_ids = pfk_ssm_sigma_ids.size();
	if(n_pfk_ssm_sigma_ids < pfk_n_layers){
		printf("Insufficient sigma IDs specified for %d layer PF: %d\n",
			pfk_n_layers, n_pfk_ssm_sigma_ids);
		return false;
	}
	//! take the last 'pfk_n_layers' ssm_sigma_ids added to pfk_ssm_sigma_ids so that the ones specified 
	//! in modules.cfg can be completely overridden at runtime by command line arguments that are parsed last
	typedef SearchMethod<AMType, SSMType> SMType;
	int start_id = n_pfk_ssm_sigma_ids - pfk_n_layers;
	for(int layer_id = start_id; layer_id < n_pfk_ssm_sigma_ids; ++layer_id){
		pf_ssm_sigma_ids = pfk_ssm_sigma_ids[layer_id];
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("pf", am_params, ssm_params)));
		if(!trackers.back()){ return false; }
	}
	return true;
}
//! Multi layer Nearest Neighbor
template< class AMType, class SSMType >
bool getNNk(vector<SearchMethod<AMType, SSMType>*> &trackers,
	const typename AMType::ParamType *am_params = nullptr,
	const typename SSMType::ParamType *ssm_params = nullptr){
	int n_nnk_ssm_sigma_ids = nnk_ssm_sigma_ids.size();
	if(n_nnk_ssm_sigma_ids < nnk_n_layers){
		printf("Insufficient sigma IDs specified for %d layer NN: %d\n",
			nnk_n_layers, n_nnk_ssm_sigma_ids);
		return false;
	}
	//! take the last 'nnk_n_layers' ssm_sigma_ids added to nnk_ssm_sigma_ids so that the ones specified 
	//! in modules.cfg can be completely overridden at runtime by command line arguments that are parsed last
	typedef SearchMethod<AMType, SSMType> SMType;
	int start_id = n_nnk_ssm_sigma_ids - nnk_n_layers;
	for(int layer_id = start_id; layer_id < n_nnk_ssm_sigma_ids; ++layer_id){
		nn_ssm_sigma_ids = nnk_ssm_sigma_ids[layer_id];
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("nn", am_params, ssm_params)));
		if(!trackers.back()){ return false; }
	}
	return true;
}
template< class AMType, class SSMType >
TrackerBase *getTracker(const char *sm_type,
	const typename AMType::ParamType *am_params,
	const typename SSMType::ParamType *ssm_params){
#ifndef DISABLE_TEMPLATED_SM
	typedef SearchMethod<AMType, SSMType> SMType;
	if(!strcmp(sm_type, "esm")){
		return new ESM<AMType, SSMType>(getESMParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "iclk") || !strcmp(sm_type, "ic")){
		return new ICLK<AMType, SSMType>(getICLKParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "fclk") || !strcmp(sm_type, "fc")){
		return new FCLK<AMType, SSMType>(getFCLKParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "falk") || !strcmp(sm_type, "fa")){
		return new FALK<AMType, SSMType>(getFALKParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "ialk") || !strcmp(sm_type, "ia")){
		return new IALK<AMType, SSMType>(getIALKParams().get(), am_params, ssm_params);
	}
	//! Levenberg Marquardt Formulations
	else if(!strcmp(sm_type, "eslm") || !strcmp(sm_type, "esl")){
		leven_marq = true;
		return new ESM<AMType, SSMType>(getESMParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "iclm") || !strcmp(sm_type, "icl")){
		leven_marq = true;
		return new ICLK<AMType, SSMType>(getICLKParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "fclm") || !strcmp(sm_type, "fcl")){
		leven_marq = true;
		return new FCLK<AMType, SSMType>(getFCLKParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "falm") || !strcmp(sm_type, "fal")){
		leven_marq = true;
		return new FALK<AMType, SSMType>(getFALKParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "ialm") || !strcmp(sm_type, "ial")){
		leven_marq = true;
		return new IALK<AMType, SSMType>(getIALKParams().get(), am_params, ssm_params);
	}
	//! Particle Filter 
	else if(!strcmp(sm_type, "pf")){
		return new PF<AMType, SSMType>(getPFParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "pf100")){// PF with 100 particles
		pf_n_particles = 100;
		return new PF<AMType, SSMType>(getPFParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "pf250")){// PF with 250 particles
		pf_n_particles = 250;
		return new PF<AMType, SSMType>(getPFParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "pf500")){// PF with 500 particles
		pf_n_particles = 500;
		return new PF<AMType, SSMType>(getPFParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "pf1k")){// PF with 1000 particles
		pf_n_particles = 1000;
		return new PF<AMType, SSMType>(getPFParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "pf2k")){// PF with 2000 particles
		pf_n_particles = 2000;
		return new PF<AMType, SSMType>(getPFParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "pf5k")){// PF with 5000 particles
		pf_n_particles = 5000;
		return new PF<AMType, SSMType>(getPFParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "pfic")){// PF + ICLK
		vector<SMType*> trackers;
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("pf", am_params, ssm_params)));
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("ic", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pffc")){// PF + FCLK
		vector<SMType*> trackers;
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("pf", am_params, ssm_params)));
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("fc", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfes")){// PF + ESM
		vector<SMType*> trackers;
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("pf", am_params, ssm_params)));
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("esm", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfrk")){// PF + RKLT
		vector<TrackerBase*> trackers;
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("pf", am_params, ssm_params)));
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("rkl", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfk")){
		vector<SMType*> trackers;
		if(!getPFk(trackers, am_params, ssm_params)){ return nullptr; }
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfkic")){// PFk + ICLK
		vector<SMType*> trackers;
		if(!getPFk(trackers, am_params, ssm_params)){ return nullptr; }
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("ic", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfkfc")){// PFk + FCLK
		vector<SMType*> trackers;
		if(!getPFk(trackers, am_params, ssm_params)){ return nullptr; }
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("fc", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfkes")){// PFk + ESM
		vector<SMType*> trackers;
		if(!getPFk(trackers, am_params, ssm_params)){ return nullptr; }
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("esm", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	}
#ifndef DISABLE_FLANN
	//! NN based composite SMs
	else if(!strcmp(sm_type, "nnic")){// NN + ICLK 	
		vector<SMType*> trackers;
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("nn", am_params, ssm_params)));
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("ic", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnfc")){// NN + FCLK 	
		vector<SMType*> trackers;
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("nn", am_params, ssm_params)));
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("fc", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnes")){// NN + ESM 	
		vector<SMType*> trackers;
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("nn", am_params, ssm_params)));
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("esm", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnrk")){// NN + RKLT
		vector<TrackerBase*> trackers;
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("nn", am_params, ssm_params)));
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("rkl", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnk")){// Multi layer NN
		vector<SMType*> trackers;
		if(!getNNk(trackers, am_params, ssm_params)){ return nullptr; }
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnkic")){// NNIC with Multi layer NN
		vector<SMType*> trackers;
		if(!getNNk(trackers, am_params, ssm_params)){ return nullptr; }
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("ic", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnkfc")){// NNFC with Multi layer NN
		vector<SMType*> trackers;
		if(!getNNk(trackers, am_params, ssm_params)){ return nullptr; }
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("fc", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnkes")){// NNES with Multi layer NN
		vector<SMType*> trackers;
		if(!getNNk(trackers, am_params, ssm_params)){ return nullptr; }
		trackers.push_back(dynamic_cast<SMType*>(getTracker<AMType, SSMType>("esm", am_params, ssm_params)));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	}
#endif
#ifndef DISABLE_GRID
	//! Grid based composite SMs
	else if(!strcmp(sm_type, "gric")){// Grid + ICLK
		vector<TrackerBase*> trackers;
		trackers.push_back(getTracker<AMType, SSMType>("grid", am_params, ssm_params));
		trackers.push_back(getTracker<AMType, SSMType>("iclk", am_params, ssm_params));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "grfc")){// Grid + FCLK
		vector<TrackerBase*> trackers;
		trackers.push_back(getTracker<AMType, SSMType>("grid", am_params, ssm_params));
		trackers.push_back(getTracker<AMType, SSMType>("fclk", am_params, ssm_params));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "gres")){// Grid + ESM
		vector<TrackerBase*> trackers;
		trackers.push_back(getTracker<AMType, SSMType>("grid", am_params, ssm_params));
		trackers.push_back(getTracker<AMType, SSMType>("esm", am_params, ssm_params));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "rklt") || !strcmp(sm_type, "rkl")){// Grid + Template tracker with SPI
		GridBase *grid_tracker = dynamic_cast<GridBase*>(getTracker<AMType, SSMType>("grid", am_params, ssm_params));
		//if(rkl_enable_spi){
		//	printf("Setting sampling resolution of the template tracker equal to the grid size: %d x %d so that SPI can be enabled.",
		//		grid_tracker->getResX(), grid_tracker->getResY());
		//	am_params->resx = grid_tracker->getResX();
		//	am_params->resy = grid_tracker->getResY();
		//}
		SMType *templ_tracker = dynamic_cast<SMType*>(getTracker<AMType, SSMType>(rkl_sm, am_params, ssm_params));
		if(!templ_tracker){
			// invalid or third party tracker has been specified in 'rkl_sm'
			throw utils::InvalidArgument("Search method provided is not compatible with RKLT");
		}
		RKLTParams rkl_params(rkl_enable_spi, rkl_enable_feedback,
			rkl_failure_detection, rkl_failure_thresh, debug_mode);
		return new RKLT<AMType, SSMType>(&rkl_params, grid_tracker, templ_tracker);
	} else if(!strcmp(sm_type, "lmes")){
		// LMS + ESM
		est_method = 1;
		GridBase *grid_tracker = dynamic_cast<GridBase*>(getTracker<AMType, SSMType>("grid", am_params, ssm_params));
		SMType *templ_tracker = dynamic_cast<SMType*>(getTracker<AMType, SSMType>("esm", am_params, ssm_params));
		if(!templ_tracker){
			//! ESM is not available
			throw utils::InvalidArgument("ESM is not available");
		}
		RKLTParams rkl_params(rkl_enable_spi, rkl_enable_feedback,
			rkl_failure_detection, rkl_failure_thresh, debug_mode);
		return new RKLT<AMType, SSMType>(&rkl_params, grid_tracker, templ_tracker);
	}
#endif
	//! cascade of search methods
	if(!strcmp(sm_type, "casm")){
		//! make copies of the original AM and SSM types in case they have 
		//! accidently been changed in the multi trcker param file since
		//! all trackers in CascadeSM must have the same AM and SSM
		std::string _mtf_am(mtf_am), _mtf_ssm(mtf_ssm);
		FILE *fid = nullptr;
		vector<SMType*> trackers;
		trackers.resize(casc_n_trackers);
		for(int i = 0; i < casc_n_trackers; i++){
			fid = readTrackerParams(fid, 1);
			if(!(trackers[i] = dynamic_cast<SMType*>(getTracker(mtf_sm, _mtf_am.c_str(),
				_mtf_ssm.c_str(), mtf_ilm)))){
				printf("Invalid search method provided for cascade SM tracker: %s\n", mtf_sm);
				return nullptr;
			}
		}
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeSM<AMType, SSMType>(trackers, &casc_params);
	}
	//! pyramidal SM
	else if(!strcmp(sm_type, "pysm") || !strcmp(sm_type, "pyrs")) {
		PyramidalParams pyr_params(pyr_no_of_levels, pyr_scale_factor, pyr_show_levels);
		vector<SMType*> trackers;
		trackers.resize(pyr_params.no_of_levels);
		if(pyr_scale_res){
			printf("Sampling resolution scaling for pyramid levels is enabled");
		}
		int resx_back = resx, resy_back = resy;
		typename AMType::ParamType _am_params(am_params);
		typename SSMType::ParamType _ssm_params(ssm_params);
		for(int tracker_id = 0; tracker_id < pyr_params.no_of_levels; tracker_id++) {
			if(enable_nt){
				printf("PyramidalSM cannot be used with NT search methods\n");
				return nullptr;
			} else{
				trackers[tracker_id] = dynamic_cast<SMType*>(getTracker<AMType, SSMType>(pyr_sm.c_str(), &_am_params, &_ssm_params));
			}
			if(!trackers[tracker_id]){
				printf("Search method provided: %s is not compatible with PyramidalSM\n", pyr_sm.c_str());
				return nullptr;
			}
			if(pyr_scale_res){
				resx = static_cast<unsigned int>(resx * pyr_scale_factor);
				resy = static_cast<unsigned int>(resy * pyr_scale_factor);
				_am_params.resx = resx;
				_am_params.resy = resy;
				_ssm_params.resx = resx;
				_ssm_params.resy = resy;
			}
		}
		resx = resx_back;
		resy = resy_back;
		return new PyramidalSM<AMType, SSMType>(trackers, &pyr_params);
	}
	//! hierarchical SSM tracker
	else if(!strcmp(sm_type, "hrch")){
		vector<TrackerBase*> trackers;
		trackers.push_back(mtf::getTracker(hrch_sm, hrch_am, "2", mtf_ilm));
		//trackers.push_back(mtf::getTrackerObj(hrch_sm, hrch_am, "3"));
		trackers.push_back(mtf::getTracker(hrch_sm, hrch_am, "4", mtf_ilm));
		trackers.push_back(mtf::getTracker(hrch_sm, hrch_am, "6", mtf_ilm));
		trackers.push_back(mtf::getTracker(hrch_sm, hrch_am, "8", mtf_ilm));

		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	}
	else if(!strcmp(sm_type, "prls") || !strcmp(sm_type, "prsm")) {// SM specific parallel tracker
		vector<SMType*> trackers;
		trackers.resize(prl_n_trackers);
		//char prl_am[strlen(mtf_am) + 1], prl_ssm[strlen(mtf_ssm) + 1];
		//strcpy(mtf_am, prl_am);
		//strcpy(mtf_ssm, prl_ssm);

		//! make copies of the original AM and SSM types in case they have 
		//! accidently been changed in the multi trcker param file since
		//! all trackers in ParallelSM must have the same AM and SSM
		std::string _mtf_am(mtf_am), _mtf_ssm(mtf_ssm);
		FILE *fid = nullptr;
		for(int tracker_id = 0; tracker_id < prl_n_trackers; tracker_id++) {
			fid = readTrackerParams(fid, 1);
			//typename AMType::ParamType _am_params(am_params);
			////! cannot change params specific to each AM and SSM but can at least change the common ones
			//_am_params.resx = resx;
			//_am_params.resy = resy;
			//_am_params.grad_eps = grad_eps;
			//_am_params.hess_eps = hess_eps;
			//typename SSMType::ParamType _ssm_params(ssm_params);
			//_ssm_params.resx = resx;
			//_ssm_params.resy = resy;
			if(!(trackers[tracker_id] = dynamic_cast<SMType*>(getTracker(mtf_sm, _mtf_am.c_str(),
				_mtf_ssm.c_str(), mtf_ilm)))) {
				printf("Invalid search method provided for parallel SM tracker: %s\n", mtf_sm);
				return nullptr;
			}
		}
		ParallelParams prl_params(
			static_cast<ParallelParams::PrlEstMethod>(prl_estimation_method),
			prl_reset_to_mean, prl_auto_reinit, prl_reinit_err_thresh, prl_reinit_frame_gap);
		return new ParallelSM<AMType, SSMType>(trackers, &prl_params, ssm_params);
	}
#endif
#ifndef DISABLE_FLANN
	//! FLANN based NN tracker
	if(!strcmp(sm_type, "nn")){// Nearest Neighbor Search
		return new NN<AMType, SSMType>(getNNParams().get(), getFLANNParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "nnkdt") || !strcmp(sm_type, "kdt")){// NN with KD Tree Index
		nn_index_type = 1;
		return new NN<AMType, SSMType>(getNNParams().get(), getFLANNParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "nnkmn") || !strcmp(sm_type, "kmn")){// NN with KMeans Clustering Index
		nn_index_type = 2;
		return new NN<AMType, SSMType>(getNNParams().get(), getFLANNParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "gnn")){// Graph based NN
		nn_fgnn_index_type = nn_index_type = 0;
		return getTracker<AMType, SSMType>("nn", am_params, ssm_params);
	} else if(!strcmp(sm_type, "fgnn")){// Graph based NN with FLANN based index
		nn_index_type = 0;
		nn_fgnn_index_type = nn_fgnn_index_type == 0 ? 2 : nn_fgnn_index_type;
		return getTracker<AMType, SSMType>("nn", am_params, ssm_params);
	} else if(!strcmp(sm_type, "nn1k")){// NN with 1000 samples
		nn_n_samples = 1000;
		return new NN<AMType, SSMType>(getNNParams().get(), getFLANNParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "nn2k")){// NN with 2000 samples
		nn_n_samples = 2000;
		return new NN<AMType, SSMType>(getNNParams().get(), getFLANNParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "nn5k")){// NN with 5000 samples
		nn_n_samples = 5000;
		return new NN<AMType, SSMType>(getNNParams().get(), getFLANNParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "nn10k")){// NN with 10000 samples
		nn_n_samples = 10000;
		return new NN<AMType, SSMType>(getNNParams().get(), getFLANNParams().get(), am_params, ssm_params);
	} else if(!strcmp(sm_type, "nn100k")){// NN with 100000 samples
		nn_n_samples = 100000;
		return new NN<AMType, SSMType>(getNNParams().get(), getFLANNParams().get(), am_params, ssm_params);
	}
#endif
#ifndef DISABLE_FEAT
	if(!strcmp(sm_type, "feat")){
		bool enable_pyr = !strcmp(grid_sm, "pyr") || !strcmp(grid_sm, "pyrt");	
		DetectorType detector_type;
		if(feat_detector_type == "none" || feat_detector_type == "-1"){
			detector_type = DetectorType::NONE;
		} else if(feat_detector_type == "orb" || feat_detector_type == "0"){
			detector_type = DetectorType::ORB;
		} else if(feat_detector_type == "brisk" || feat_detector_type == "1"){
			detector_type = DetectorType::BRISK;
		} else if(feat_detector_type == "sift" || feat_detector_type == "2"){
			detector_type = DetectorType::SIFT;
		} else if(feat_detector_type == "surf" || feat_detector_type == "3"){
			detector_type = DetectorType::SURF;
		} else if(feat_detector_type == "fast" || feat_detector_type == "4"){
			detector_type = DetectorType::FAST;
		} else if(feat_detector_type == "mser" || feat_detector_type == "5"){
			detector_type = DetectorType::MSER;
		} else if(feat_detector_type == "gftt" || feat_detector_type == "6"){
			detector_type = DetectorType::GFTT;
		} else if(feat_detector_type == "agast" || feat_detector_type == "7"){
			detector_type = DetectorType::AGAST;
		} else if(feat_detector_type == "star" || feat_detector_type == "8"){
			detector_type = DetectorType::Star;
		} else if(feat_detector_type == "msd" || feat_detector_type == "9"){
			detector_type = DetectorType::MSD;
		} else{
			throw utils::InvalidArgument(cv::format("Invalid detector type provided: %s", feat_detector_type.c_str()));
		}
		DescriptorType descriptor_type;
		if(feat_descriptor_type == "orb" || feat_descriptor_type == "0"){
			descriptor_type = DescriptorType::ORB;
		} else if(feat_descriptor_type == "brisk" || feat_descriptor_type == "brk" || feat_descriptor_type == "1"){
			descriptor_type = DescriptorType::BRISK;
		} else if(feat_descriptor_type == "sift" || feat_descriptor_type == "2"){
			descriptor_type = DescriptorType::ORB;
		} else if(feat_descriptor_type == "surf" || feat_descriptor_type == "3"){
			descriptor_type = DescriptorType::ORB;
		} else if(feat_descriptor_type == "brief" || feat_descriptor_type == "brf" || feat_descriptor_type == "4"){
			descriptor_type = DescriptorType::BRIEF;
		} else if(feat_descriptor_type == "freak" || feat_descriptor_type == "frk" || feat_descriptor_type == "5"){
			descriptor_type = DescriptorType::FREAK;
		} else if(feat_descriptor_type == "lucid" || feat_descriptor_type == "lcd" || feat_descriptor_type == "6"){
			descriptor_type = DescriptorType::LUCID;
		} else if(feat_descriptor_type == "latch" || feat_descriptor_type == "lth" || feat_descriptor_type == "7"){
			descriptor_type = DescriptorType::LATCH;
		} else if(feat_descriptor_type == "daisy" || feat_descriptor_type == "dsy" || feat_descriptor_type == "8"){
			descriptor_type = DescriptorType::DAISY;
		} else if(feat_descriptor_type == "vgg" || feat_descriptor_type == "9"){
			descriptor_type = DescriptorType::VGG;
		} else if(feat_descriptor_type == "boost" || feat_descriptor_type == "bd" || feat_descriptor_type == "10"){
			descriptor_type = DescriptorType::BoostDesc;
		} else{
			throw utils::InvalidArgument(cv::format("Invalid descriptor type provided: %s", feat_descriptor_type.c_str()));
		}
		FeatureTrackerParams feat_params(
			detector_type, descriptor_type,
			getDetectorParams(detector_type), getDescriptorParams(descriptor_type),
			grid_res, grid_res, grid_patch_size, grid_patch_size, grid_reset_at_each_frame,
			feat_rebuild_index, max_iters, epsilon, enable_pyr, feat_use_cv_flann,
			feat_max_dist_ratio, feat_min_matches, uchar_input, feat_show_keypoints,
			feat_show_matches, feat_debug_mode);
		typename SSMType::ParamType _ssm_params(ssm_params);
		_ssm_params.resx = feat_params.getResX();
		_ssm_params.resy = feat_params.getResY();
		return new FeatureTracker<SSMType>(
			&feat_params, 
#ifndef DISABLE_FLANN
			getFLANNParams().get(), 
#endif
			getFLANNCVParams().get(),
			getSSMEstParams().get(),
			&_ssm_params);
	}
#endif
#ifndef DISABLE_GRID
	//! Grid Tracker
	if(!strcmp(sm_type, "grid")){
		if(!strcmp(grid_sm, "cv")){
			int input_type = grid_rgb_input ? uchar_input ? CV_8UC3 : CV_32FC3 :
				uchar_input ? CV_8UC1 : CV_32FC1;
			GridTrackerCVParams grid_params(
				grid_res, grid_res, grid_patch_size, grid_patch_size,
				grid_reset_at_each_frame, grid_patch_centroid_inside,
				grid_fb_err_thresh, grid_pyramid_levels, grid_use_min_eig_vals,
				grid_min_eig_thresh, max_iters, epsilon, input_type,
				grid_show_trackers, debug_mode);
			typename SSMType::ParamType _ssm_params(ssm_params);
			_ssm_params.resx = grid_params.getResX();
			_ssm_params.resy = grid_params.getResY();
			return new GridTrackerCV<SSMType>(&grid_params, getSSMEstParams().get(), &_ssm_params);
		}
#ifndef DISABLE_TEMPLATED_SM
		else if(!strcmp(grid_sm, "flow")){
			GridTrackerFlowParams grid_params(
				grid_res, grid_res, grid_patch_size, grid_patch_size,
				grid_pyramid_levels, grid_use_const_grad, grid_min_eig_thresh,
				max_iters, epsilon, grid_show_trackers, debug_mode);
			resx = grid_params.getResX();
			resy = grid_params.getResY();
			typename SSMType::ParamType _ssm_params(ssm_params);
			_ssm_params.resx = grid_params.getResX();
			_ssm_params.resy = grid_params.getResY();
			return new GridTrackerFlow<AMType, SSMType>(&grid_params, getSSMEstParams().get(), am_params, &_ssm_params);
		}
#endif
		else{
			vector<TrackerBase*> trackers;
			int grid_n_trackers = grid_res*grid_res;
			int resx_back = resx, resy_back = resy;
			if(grid_patch_res > 0){
				resx = resy = grid_patch_res;
			} else{
				resx = resy = grid_patch_size;
			}
			for(int tracker_id = 0; tracker_id < grid_n_trackers; tracker_id++){
				trackers.push_back(getTracker(grid_sm, grid_am, grid_ssm, grid_ilm));
				if(!trackers.back()){ return nullptr; }
			}
			resx = resx_back;
			resy = resy_back;
			bool enable_pyr = !strcmp(grid_sm, "pyr") || !strcmp(grid_sm, "pyrt");
			GridTrackerParams grid_params(
				grid_res, grid_res, grid_patch_size, grid_patch_size,
				grid_reset_at_each_frame, grid_dyn_patch_size, grid_patch_centroid_inside,
				grid_fb_err_thresh, grid_fb_reinit, grid_use_tbb, max_iters, epsilon, enable_pyr,
				grid_show_trackers, grid_show_tracker_edges, debug_mode);
			typename SSMType::ParamType _ssm_params(ssm_params);
			_ssm_params.resx = grid_params.getResX();
			_ssm_params.resy = grid_params.getResY();
			return new GridTracker<SSMType>(trackers, &grid_params, getSSMEstParams().get(), &_ssm_params);
		}
	} else if(!strcmp(sm_type, "lms")){
		est_method = 1;
		return getTracker<AMType, SSMType>("grid", am_params, ssm_params);
	} else if(!strcmp(sm_type, "ransac") || !strcmp(sm_type, "rnsc")){
		est_method = 0;
		return getTracker<AMType, SSMType>("grid", am_params, ssm_params);
	}
#endif
	//! non templated composite SMs/trackers
	if(!strcmp(sm_type, "casc")){// general purpose cascaded tracker 
		FILE *fid = nullptr;
		vector<TrackerBase*> trackers;
		trackers.resize(casc_n_trackers);
		for(int i = 0; i < casc_n_trackers; i++){
			fid = readTrackerParams(fid, 1);
			if(!(trackers[i] = getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm))){
				return nullptr;
			}
		}
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	}
	// Parallel Tracker
	else if(!strcmp(sm_type, "prl") || !strcmp(sm_type, "prlt")) { // general purpose parallel tracker 
		vector<TrackerBase*> trackers;
		trackers.resize(prl_n_trackers);
		FILE *fid = nullptr;
		for(int tracker_id = 0; tracker_id < prl_n_trackers; tracker_id++) {
			fid = readTrackerParams(fid, 1);
			if(!(trackers[tracker_id] = getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm))) {
				return nullptr;
			}
		}
		ParallelParams prl_params(
			static_cast<ParallelParams::PrlEstMethod>(prl_estimation_method),
			prl_reset_to_mean, prl_auto_reinit, prl_reinit_err_thresh, prl_reinit_frame_gap);
		return new ParallelTracker(trackers, &prl_params);
	} else if(!strcmp(sm_type, "pyr") || !strcmp(sm_type, "pyrt")) { // pyramidal tracker
		PyramidalParams pyr_params(pyr_no_of_levels, pyr_scale_factor, pyr_show_levels);
		vector<TrackerBase*> trackers;
		trackers.resize(pyr_params.no_of_levels);
		if(pyr_scale_res){
			printf("Sampling resolution scaling for pyramid levels is enabled");
		}
		int resx_back = resx, resy_back = resy;
		typename AMType::ParamType _am_params(am_params);
		typename SSMType::ParamType _ssm_params(ssm_params);
		for(int tracker_id = 0; tracker_id < pyr_params.no_of_levels; ++tracker_id) {
			trackers[tracker_id] = enable_nt ? getTracker(pyr_sm.c_str(), mtf_am, mtf_ssm, mtf_ilm) :
				getTracker<AMType, SSMType>(pyr_sm.c_str(), &_am_params, &_ssm_params);
			if(!trackers[tracker_id]){ return nullptr; }
			if(pyr_scale_res){
				resx = static_cast<unsigned int>(resx * pyr_scale_factor);
				resy = static_cast<unsigned int>(resy * pyr_scale_factor);
				_am_params.resx = resx;
				_am_params.resy = resy;
				_ssm_params.resx = resx;
				_ssm_params.resy = resy;
			}
		}
		resx = resx_back;
		resy = resy_back;
		return new PyramidalTracker(trackers, &pyr_params);
	} else if(!strcmp(sm_type, "line")){
		vector<TrackerBase*> trackers;
		int line_n_trackers = line_grid_size*line_grid_size;
		trackers.resize(line_n_trackers);
		int resx_back = resx, resy_back = resy;
		resx = resy = line_patch_size;
		for(int tracker_id = 0; tracker_id < line_n_trackers; tracker_id++){
			if(!(trackers[tracker_id] = getTracker(line_sm, line_am, line_ssm, mtf_ilm))){
				return nullptr;
			}
		}
		resx = resx_back;
		resy = resy_back;
		LineTrackerParams line_params(line_grid_size, line_grid_size,
			line_patch_size, line_use_constant_slope, line_use_ls,
			line_inter_alpha_thresh, line_intra_alpha_thresh,
			line_reset_pos, line_reset_template, line_debug_mode);
		return new LineTracker(trackers, &line_params);
	}
	printf("Invalid search method provided: %s\n", sm_type);
	return nullptr;
}

template< class AMType >
TrackerBase *getTracker(const char *sm_type, const char *ssm_type,
	const typename AMType::ParamType *am_params = nullptr){

	SSMParams_ params = getSSMParams(ssm_type);
	if(!params){ return nullptr; }

	if(!strcmp(ssm_type, "lhom") || !strcmp(ssm_type, "l8")){
		return getTracker<AMType, LieHomography>(sm_type, am_params, cast_params(LieHomography));
	} else if(!strcmp(ssm_type, "cbh") || !strcmp(ssm_type, "c8")){
		return getTracker<AMType, mtf::CBH>(sm_type, am_params, cast_params(CBH));
	} else if(!strcmp(ssm_type, "sl3")){
		return getTracker<AMType, mtf::SL3>(sm_type, am_params, cast_params(SL3));
	} else if(!strcmp(ssm_type, "hom") || !strcmp(ssm_type, "8")){
		return getTracker<AMType, mtf::Homography>(sm_type, am_params, cast_params(Homography));
	} else if(!strcmp(ssm_type, "aff") || !strcmp(ssm_type, "6")){
		return getTracker<AMType, mtf::Affine>(sm_type, am_params, cast_params(Affine));
	} else if(!strcmp(ssm_type, "laff") || !strcmp(ssm_type, "l6")){
		return getTracker<AMType, mtf::LieAffine>(sm_type, am_params, cast_params(LieAffine));
	} else if(!strcmp(ssm_type, "asrt") || !strcmp(ssm_type, "5")){
		return getTracker<AMType, mtf::ASRT>(sm_type, am_params, cast_params(ASRT));
	} else if(!strcmp(ssm_type, "sim") || !strcmp(ssm_type, "4")){
		return getTracker<AMType, mtf::Similitude>(sm_type, am_params, cast_params(Similitude));
	} else if(!strcmp(ssm_type, "iso") || !strcmp(ssm_type, "3")){
		return getTracker<AMType, mtf::Isometry>(sm_type, am_params, cast_params(Isometry));
	} else if(!strcmp(ssm_type, "ast") || !strcmp(ssm_type, "4s")){
		return getTracker<AMType, mtf::AST>(sm_type, am_params, cast_params(AST));
	} else if(!strcmp(ssm_type, "ist") || !strcmp(ssm_type, "3s")){
		return getTracker<AMType, mtf::IST>(sm_type, am_params, cast_params(IST));
	} else if(!strcmp(ssm_type, "trans") || !strcmp(ssm_type, "2")){
		return getTracker<AMType, mtf::Translation>(sm_type, am_params, cast_params(Translation));
	} else if(!strcmp(ssm_type, "spline") || !strcmp(ssm_type, "spl")){
		return getTracker<AMType, mtf::Spline>(sm_type, am_params, cast_params(Spline));
	} else{
		printf("Invalid state space model provided: %s\n", ssm_type);
		return nullptr;
	}
}
//! main function for creating trackers
inline TrackerBase *getTracker(const char *sm_type, const char *am_type,
	const char *ssm_type, const char *ilm_type){
//#ifdef DISABLE_TEMPLATED_SM
//	enable_nt = 1;
//#endif
	// check for 3rd party trackers
	TrackerBase *third_party_tracker = getTracker(sm_type);
	if(third_party_tracker)
		return third_party_tracker;
	// check if Non templated variant is needed
	if(enable_nt){
		TrackerBase *nt_tracker = getCompositeSM(sm_type, am_type, ssm_type, ilm_type);
		if(nt_tracker){ return nt_tracker; }
		nt_tracker = getSM(sm_type, am_type, ssm_type, ilm_type);
		if(nt_tracker){ return nt_tracker; }
	}

	AMParams_ params = getAMParams(am_type, ilm_type);
	if(!params){ return nullptr; }

	if(!strcmp(am_type, "ssd")){
		return getTracker<SSD>(sm_type, ssm_type, cast_params(SSD));
	} else if(!strcmp(am_type, "nssd")){
		return getTracker<NSSD>(sm_type, ssm_type, cast_params(NSSD));
	} else if(!strcmp(am_type, "zncc")){
		return getTracker<ZNCC>(sm_type, ssm_type, cast_params(ZNCC));
	} else if(!strcmp(am_type, "scv")){
		return getTracker<SCV>(sm_type, ssm_type, cast_params(SCV));
	} else if(!strcmp(am_type, "lscv")){
		return getTracker<LSCV>(sm_type, ssm_type, cast_params(LSCV));
	} else if(!strcmp(am_type, "rscv")){
		return getTracker<RSCV>(sm_type, ssm_type, cast_params(RSCV));
	} else if(!strcmp(am_type, "lrscv") || !strcmp(am_type, "lrsc")){
		return getTracker<LRSCV>(sm_type, ssm_type, cast_params(LRSCV));
	} else if(!strcmp(am_type, "kld")){
		return getTracker<KLD>(sm_type, ssm_type, cast_params(KLD));
	} else if(!strcmp(am_type, "lkld")){
		return getTracker<LKLD>(sm_type, ssm_type, cast_params(LKLD));
	} else if(!strcmp(am_type, "mi")){
		return getTracker<MI>(sm_type, ssm_type, cast_params(MI));
	} else if(!strcmp(am_type, "spss")){
		return getTracker<SPSS>(sm_type, ssm_type, cast_params(SPSS));
	} else if(!strcmp(am_type, "ssim")){
		return getTracker<SSIM>(sm_type, ssm_type, cast_params(SSIM));
	} else if(!strcmp(am_type, "ncc")){
		return getTracker<NCC>(sm_type, ssm_type, cast_params(NCC));
	} else if(!strcmp(am_type, "ccre")){
		return getTracker<CCRE>(sm_type, ssm_type, cast_params(CCRE));
	} else if(!strcmp(am_type, "riu")){
		return getTracker<RIU>(sm_type, ssm_type, cast_params(RIU));
	} else if(!strcmp(am_type, "ngf")){
		return getTracker<NGF>(sm_type, ssm_type, cast_params(NGF));
	} else if(!strcmp(am_type, "mcssd") || !strcmp(am_type, "ssd3")){
		return getTracker<MCSSD>(sm_type, ssm_type, cast_params(MCSSD));
	} else if(!strcmp(am_type, "mcncc") || !strcmp(am_type, "ncc3")){
		return getTracker<MCNCC>(sm_type, ssm_type, cast_params(MCNCC));
	} else if(!strcmp(am_type, "mcmi") || !strcmp(am_type, "mi3")){
		return getTracker<MCMI>(sm_type, ssm_type, cast_params(MCMI));
	} else if(!strcmp(am_type, "mczncc") || !strcmp(am_type, "zncc3")){
		return getTracker<MCZNCC>(sm_type, ssm_type, cast_params(MCZNCC));
	} else if(!strcmp(am_type, "mcscv") || !strcmp(am_type, "scv3")){
		return getTracker<MCSCV>(sm_type, ssm_type, cast_params(MCSCV));
	} else if(!strcmp(am_type, "mclscv") || !strcmp(am_type, "lscv3")){
		return getTracker<MCLSCV>(sm_type, ssm_type, cast_params(MCLSCV));
	} else if(!strcmp(am_type, "mcrscv") || !strcmp(am_type, "rscv3")){
		return getTracker<MCRSCV>(sm_type, ssm_type, cast_params(MCRSCV));
	} else if(!strcmp(am_type, "mcssim") || !strcmp(am_type, "ssim3")){
		return getTracker<MCSSIM>(sm_type, ssm_type, cast_params(MCSSIM));
	} else if(!strcmp(am_type, "mcspss") || !strcmp(am_type, "spss3")){
		return getTracker<MCSPSS>(sm_type, ssm_type, cast_params(MCSPSS));
	} else if(!strcmp(am_type, "mcccre") || !strcmp(am_type, "ccre3")){
		return getTracker<MCCCRE>(sm_type, ssm_type, cast_params(MCCCRE));
	} else if(!strcmp(am_type, "mcriu") || !strcmp(am_type, "riu3")){
		return getTracker<MCRIU>(sm_type, ssm_type, cast_params(MCRIU));
	}
#ifndef DISABLE_DFM
	else if(!strcmp(am_type, "dfm")){
		return getTracker<DFM>(sm_type, ssm_type, cast_params(DFM));
	}
#endif	
#ifndef DISABLE_PCA
	else if(!strcmp(am_type, "pca")){
		enable_learning = true;
		return getTracker<PCA>(sm_type, ssm_type, cast_params(PCA));
	} else if(!strcmp(am_type, "mcpca") || !strcmp(am_type, "pca3")){
		enable_learning = true;
		return getTracker<MCPCA>(sm_type, ssm_type, cast_params(MCPCA));
	}
#endif	
	else{
		printf("Invalid appearance model provided: %s\n", am_type);
		return nullptr;
	}
}
inline SSMParams_ getSSMParams(const char *ssm_type){
	SSMParams_ ssm_params(new SSMParams(resx, resy));
	if(!strcmp(ssm_type, "lhom") || !strcmp(ssm_type, "l8")){
		return SSMParams_(new LieHomographyParams(ssm_params.get(), lhom_normalized_init,
			lhom_grad_eps, debug_mode));
	} else if(!strcmp(ssm_type, "cbh") || !strcmp(ssm_type, "c8")){
		return SSMParams_(new CBHParams(ssm_params.get(), cbh_normalized_init,
			cbh_grad_eps, debug_mode));
	} else if(!strcmp(ssm_type, "sl3")){
		return SSMParams_(new SL3Params(ssm_params.get(), sl3_normalized_init, sl3_iterative_sample_mean,
			sl3_sample_mean_max_iters, sl3_sample_mean_eps, sl3_debug_mode));
	} else if(!strcmp(ssm_type, "hom") || !strcmp(ssm_type, "8")){
		return SSMParams_(new HomographyParams(ssm_params.get(), hom_normalized_init,
			hom_corner_based_sampling, debug_mode));
	} else if(!strcmp(ssm_type, "aff") || !strcmp(ssm_type, "6")){
		return SSMParams_(new AffineParams(ssm_params.get(), aff_normalized_init, aff_pt_based_sampling, debug_mode));
	} else if(!strcmp(ssm_type, "laff") || !strcmp(ssm_type, "l6")){
		return SSMParams_(new LieAffineParams(ssm_params.get(), laff_normalized_init, laff_grad_eps, debug_mode));
	} else if(!strcmp(ssm_type, "asrt") || !strcmp(ssm_type, "5")){
		return SSMParams_(new ASRTParams(ssm_params.get(), asrt_normalized_init,
			asrt_pt_based_sampling, debug_mode));
	} else if(!strcmp(ssm_type, "sim") || !strcmp(ssm_type, "4")){
		return SSMParams_(new SimilitudeParams(ssm_params.get(), sim_normalized_init,
			sim_geom_sampling, sim_pt_based_sampling, sim_n_model_pts, debug_mode));
	} else if(!strcmp(ssm_type, "iso") || !strcmp(ssm_type, "3")){
		return SSMParams_(new IsometryParams(ssm_params.get(), iso_pt_based_sampling));
	} else if(!strcmp(ssm_type, "ast") || !strcmp(ssm_type, "4s")){
		return SSMParams_(new ASTParams(ssm_params.get(), debug_mode));
	} else if(!strcmp(ssm_type, "ist") || !strcmp(ssm_type, "3s")){
		return SSMParams_(new ISTParams(ssm_params.get(), debug_mode));
	} else if(!strcmp(ssm_type, "trans") || !strcmp(ssm_type, "2")){
		return SSMParams_(new TranslationParams(ssm_params.get(), debug_mode));
	} else if(!strcmp(ssm_type, "spline") || !strcmp(ssm_type, "spl")){
		return SSMParams_(new SplineParams(ssm_params.get(), spl_control_size, spl_control_size,
			spl_control_overlap,
			static_cast<SplineParams::InterpolationType>(spl_interp_type),
			spl_static_wts, spl_debug_mode));
	} else{
		printf("Invalid state space model provided: %s\n", ssm_type);
		return nullptr;
	}
}
//! SSMs for non templated SMs
inline StateSpaceModel *getSSM(const char *ssm_type){
	SSMParams_ params = getSSMParams(ssm_type);
	if(!params){ return nullptr; }

	if(!strcmp(ssm_type, "lhom") || !strcmp(ssm_type, "l8")){
		return new LieHomography(cast_params(LieHomography));
	} else if(!strcmp(ssm_type, "cbh") || !strcmp(ssm_type, "c8")){
		return new mtf::CBH(cast_params(CBH));
	} else if(!strcmp(ssm_type, "sl3")){
		return new mtf::SL3(cast_params(SL3));
	} else if(!strcmp(ssm_type, "hom") || !strcmp(ssm_type, "8")){
		return new mtf::Homography(cast_params(Homography));
	} else if(!strcmp(ssm_type, "aff") || !strcmp(ssm_type, "6")){
		return new mtf::Affine(cast_params(Affine));
	} else if(!strcmp(ssm_type, "laff") || !strcmp(ssm_type, "l6")){
		return new mtf::LieAffine(cast_params(LieAffine));
	} else if(!strcmp(ssm_type, "asrt") || !strcmp(ssm_type, "5")){
		return new mtf::ASRT(cast_params(ASRT));
	} else if(!strcmp(ssm_type, "sim") || !strcmp(ssm_type, "4")){
		return new mtf::Similitude(cast_params(Similitude));
	} else if(!strcmp(ssm_type, "iso") || !strcmp(ssm_type, "3")){
		return new mtf::Isometry(cast_params(Isometry));
	} else if(!strcmp(ssm_type, "ast") || !strcmp(ssm_type, "4s")){
		return new mtf::AST(cast_params(AST));
	} else if(!strcmp(ssm_type, "ist") || !strcmp(ssm_type, "3s")){
		return new mtf::IST(cast_params(IST));
	} else if(!strcmp(ssm_type, "trans") || !strcmp(ssm_type, "2")){
		return new mtf::Translation(cast_params(Translation));
	} else if(!strcmp(ssm_type, "spline") || !strcmp(ssm_type, "spl")){
		return new mtf::Spline(cast_params(Spline));
	} else{
		printf("Invalid state space model provided: %s\n", ssm_type);
		return nullptr;
	}
}
//! Illumination models for AMs
inline IlluminationModel *getILM(const char *ilm_type){
	ILMParams ilm_params(resx, resy);
	if(!strcmp(ilm_type, "gb")){
		GBParams gb_params(&ilm_params, gb_additive_update);
		return new GB(&gb_params);
	} else if(!strcmp(ilm_type, "pgb")){
		PGBParams pgb_params(&ilm_params, pgb_additive_update,
			pgb_sub_regions_x, pgb_sub_regions_y);
		return new PGB(&pgb_params);
	} else if(!strcmp(ilm_type, "rbf")){
		RBFParams rbf_params(&ilm_params, rbf_additive_update,
			rbf_n_ctrl_pts_x, rbf_n_ctrl_pts_y);
		return new RBF(&rbf_params);
	} else{
		return nullptr;
	}
}

inline AMParams_ getAMParams(const char *am_type, const char *ilm_type){
	AMParams_ am_params(new AMParams(resx, resy, grad_eps, hess_eps,
		uchar_input, likelihood_alpha, likelihood_beta, dist_from_likelihood,
		learning_rate, getILM(ilm_type)));
	if(!strcmp(am_type, "ssd") || !strcmp(am_type, "mcssd") || !strcmp(am_type, "ssd3")){
		return AMParams_(new SSDParams(am_params.get(), ssd_show_template));
	} else if(!strcmp(am_type, "sad") || !strcmp(am_type, "mcsad") || !strcmp(am_type, "sad3")){
		return am_params;
	} else if(!strcmp(am_type, "nssd")){
		return AMParams_(new NSSDParams(am_params.get(), nssd_norm_pix_max, nssd_norm_pix_min, debug_mode));
	} else if(!strcmp(am_type, "zncc") || !strcmp(am_type, "mczncc") || !strcmp(am_type, "zncc3")){
		return AMParams_(new ZNCCParams(am_params.get(), debug_mode));
	} else if(!strcmp(am_type, "scv") || !strcmp(am_type, "mcscv") || !strcmp(am_type, "scv3")){
		return AMParams_(new SCVParams(am_params.get(), static_cast<SCVParams::HistType>(scv_hist_type),
			scv_n_bins, scv_preseed, scv_pou, scv_weighted_mapping, scv_mapped_gradient,
			scv_approx_dist_feat, debug_mode));
	} else if(!strcmp(am_type, "lscv") || !strcmp(am_type, "mclscv") || !strcmp(am_type, "lscv3")){
		return AMParams_(new LSCVParams(am_params.get(), lscv_sub_regions, lscv_sub_regions,
			lscv_spacing, lscv_spacing, scv_affine_mapping, scv_once_per_frame, scv_n_bins, scv_preseed,
			scv_weighted_mapping, lscv_show_subregions, scv_approx_dist_feat));
	} else if(!strcmp(am_type, "rscv") || !strcmp(am_type, "mcrscv") || !strcmp(am_type, "rscv3")){
		return AMParams_(new RSCVParams(am_params.get(), scv_use_bspl, scv_n_bins, scv_preseed, scv_pou,
			scv_weighted_mapping, scv_mapped_gradient, scv_approx_dist_feat, debug_mode));
	} else if(!strcmp(am_type, "lrscv") || !strcmp(am_type, "lrsc")){
		return AMParams_(new LRSCVParams(am_params.get(), lscv_sub_regions, lscv_sub_regions,
			lscv_spacing, lscv_spacing, scv_affine_mapping, scv_once_per_frame, scv_n_bins, scv_preseed,
			scv_weighted_mapping, lscv_show_subregions, debug_mode));
	} else if(!strcmp(am_type, "kld")){
		return AMParams_(new KLDParams(am_params.get(), mi_n_bins, mi_pre_seed, mi_pou, debug_mode));
	} else if(!strcmp(am_type, "lkld")){
		return AMParams_(new LKLDParams(am_params.get(), lkld_sub_regions, lkld_sub_regions,
			lkld_spacing, lkld_spacing, lkld_n_bins, lkld_pre_seed, lkld_pou, debug_mode));
	} else if(!strcmp(am_type, "mi") || !strcmp(am_type, "mcmi") || !strcmp(am_type, "mi3")){
		return AMParams_(new MIParams(am_params.get(), mi_n_bins, mi_pre_seed, mi_pou,
			getPixMapper(pix_mapper), debug_mode));
	} else if(!strcmp(am_type, "spss") || !strcmp(am_type, "mcspss") || !strcmp(am_type, "spss3")){
		return AMParams_(new SPSSParams(am_params.get(), spss_k,
			getPixMapper(pix_mapper)));
	} else if(!strcmp(am_type, "ssim") || !strcmp(am_type, "mcssim") || !strcmp(am_type, "ssim3")){
		return AMParams_(new SSIMParams(am_params.get(), ssim_k1, ssim_k2));
	} else if(!strcmp(am_type, "ncc") || !strcmp(am_type, "mcncc") || !strcmp(am_type, "ncc3")){
		return AMParams_(new NCCParams(am_params.get(), ncc_fast_hess));
	} else if(!strcmp(am_type, "ccre") || !strcmp(am_type, "mcccre") || !strcmp(am_type, "ccre3")){
		return AMParams_(new CCREParams(am_params.get(), ccre_n_bins, ccre_pou, ccre_pre_seed,
			ccre_symmetrical_grad, ccre_n_blocks, debug_mode));
	} else if(!strcmp(am_type, "riu") || !strcmp(am_type, "mcriu") || !strcmp(am_type, "riu3")){
		return AMParams_(new RIUParams(am_params.get(), debug_mode));
	} else if(!strcmp(am_type, "ngf")){
		return AMParams_(new NGFParams(am_params.get(), ngf_eta, ngf_use_ssd));
	} else if(!strcmp(am_type, "sum")){
		return am_params;
	}
#ifndef DISABLE_DFM
	else if(!strcmp(am_type, "dfm")){
		return AMParams_(new DFMParams(am_params.get(), dfm_nfmaps, dfm_layer_name,
			dfm_vis, dfm_zncc, dfm_model_f_name, dfm_mean_f_name, dfm_params_f_name));
	}
#endif	
#ifndef DISABLE_PCA
	else if(!strcmp(am_type, "pca") || !strcmp(am_type, "mcpca") || !strcmp(am_type, "pca3")){
		return AMParams_(new PCAParams(am_params.get(), pca_n_eigenvec,
			pca_batchsize, pca_f_factor, pca_show_basis));
	}
#endif	
	else{
		printf("Invalid appearance model provided: %s\n", am_type);
		return nullptr;
	}
}

//! AMs for non templated SMs
inline AppearanceModel *getAM(const char *am_type, const char *ilm_type){
	AMParams_ params = getAMParams(am_type, ilm_type);
	if(!params){ return nullptr; }

	//AMParams am_params(resx, resy, grad_eps, hess_eps, likelihood_alpha, getILM(ilm_type));
	if(!strcmp(am_type, "ssd")){
		return new SSD(cast_params(SSD));
	}if(!strcmp(am_type, "sad")){
		return new SAD(cast_params(SAD));
	} else if(!strcmp(am_type, "nssd")){
		return new NSSD(cast_params(NSSD));
	} else if(!strcmp(am_type, "zncc")){
		return new ZNCC(cast_params(ZNCC));
	} else if(!strcmp(am_type, "scv")){
		return new SCV(cast_params(SCV));
	} else if(!strcmp(am_type, "lscv")){
		return new LSCV(cast_params(LSCV));
	} else if(!strcmp(am_type, "rscv")){
		return new RSCV(cast_params(RSCV));
	} else if(!strcmp(am_type, "lrscv") || !strcmp(am_type, "lrsc")){
		return new LRSCV(cast_params(LRSCV));
	} else if(!strcmp(am_type, "kld")){
		return new KLD(cast_params(KLD));
	} else if(!strcmp(am_type, "lkld")){
		return new LKLD(cast_params(LKLD));
	} else if(!strcmp(am_type, "mi")){
		return new MI(cast_params(MI));
	} else if(!strcmp(am_type, "spss")){
		return new SPSS(cast_params(SPSS));
	} else if(!strcmp(am_type, "ssim")){
		return new SSIM(cast_params(SSIM));
	} else if(!strcmp(am_type, "ncc")){
		return new NCC(cast_params(NCC));
	} else if(!strcmp(am_type, "ccre")){
		return new CCRE(cast_params(CCRE));
	} else if(!strcmp(am_type, "riu")){
		return new RIU(cast_params(RIU));
	} else if(!strcmp(am_type, "ngf")){
		return new NGF(cast_params(NGF));
	}
	//! composite AMs
	else if(!strcmp(am_type, "sum")){
		return new SumOfAMs(getAM(sum_am1.c_str(), mtf_ilm),
			getAM(sum_am2.c_str(), mtf_ilm), params.get());
	}
	//! multi channel variants
	else if(!strcmp(am_type, "mcssd") || !strcmp(am_type, "ssd3")){
		return new MCSSD(cast_params(MCSSD));
	} else if(!strcmp(am_type, "mcsad") || !strcmp(am_type, "sad3")){
		return new MCSAD(cast_params(MCSAD));
	} else if(!strcmp(am_type, "mczncc") || !strcmp(am_type, "zncc3")){
		return new MCZNCC(cast_params(MCZNCC));
	} else if(!strcmp(am_type, "mcscv") || !strcmp(am_type, "scv3")){
		return new MCSCV(cast_params(MCSCV));
	} else if(!strcmp(am_type, "mclscv") || !strcmp(am_type, "lscv3")){
		return new MCLSCV(cast_params(MCLSCV));
	} else if(!strcmp(am_type, "mcrscv") || !strcmp(am_type, "rscv3")){
		return new MCRSCV(cast_params(MCRSCV));
	} else if(!strcmp(am_type, "mcncc") || !strcmp(am_type, "ncc3")){
		return new MCNCC(cast_params(MCNCC));
	} else if(!strcmp(am_type, "mcmi") || !strcmp(am_type, "mi3")){
		return new MCMI(cast_params(MCMI));
	} else if(!strcmp(am_type, "mcssim") || !strcmp(am_type, "ssim3")){
		return new MCSSIM(cast_params(MCSSIM));
	} else if(!strcmp(am_type, "mcspss") || !strcmp(am_type, "spss3")){
		return new MCSPSS(cast_params(MCSPSS));
	} else if(!strcmp(am_type, "mcccre") || !strcmp(am_type, "ccre3")){
		return new MCCCRE(cast_params(MCCCRE));
	} else if(!strcmp(am_type, "mcriu") || !strcmp(am_type, "riu3")){
		return new MCRIU(cast_params(MCRIU));
	}
#ifndef DISABLE_DFM
	else if(!strcmp(am_type, "dfm")){
		return new DFM(cast_params(DFM));
	}
#endif	
#ifndef DISABLE_PCA
	else if(!strcmp(am_type, "pca")){
		enable_learning = true;
		return new PCA(cast_params(PCA));
	} else if(!strcmp(am_type, "mcpca") || !strcmp(am_type, "pca3")){
		enable_learning = true;
		return new MCPCA(cast_params(MCPCA));
	}
#endif	
	else{
		printf("Invalid appearance model provided: %s\n", am_type);
		return nullptr;
	}
}
//! non templated search methods
inline nt::SearchMethod *getSM(const char *sm_type,
	const char *am_type, const char *ssm_type, const char *ilm_type){
	AM am(getAM(am_type, ilm_type));
	SSM ssm(getSSM(ssm_type));
	if(!am || !ssm){ return nullptr; }
	if(!strcmp(sm_type, "fclk") || !strcmp(sm_type, "fc")){
		return new nt::FCLK(am, ssm, getFCLKParams().get());
	} else  if(!strcmp(sm_type, "esm")){
		return new nt::ESM(am, ssm, getESMParams().get());
	} else if(!strcmp(sm_type, "aesm")){
		return new nt::AESM(am, ssm, getESMParams().get());
	} else if(!strcmp(sm_type, "iclk") || !strcmp(sm_type, "ic")){
		return new nt::ICLK(am, ssm, getICLKParams().get());
	} else if(!strcmp(sm_type, "falk") || !strcmp(sm_type, "fa")){
		return new nt::FALK(am, ssm, getFALKParams().get());
	} else if(!strcmp(sm_type, "ialk") || !strcmp(sm_type, "ia")){
		return new nt::IALK(am, ssm, getIALKParams().get());
	}
	//! Levenberg Marquardt Formulations
	else if(!strcmp(sm_type, "eslm") || !strcmp(sm_type, "esl")){
		leven_marq = true;
		return new nt::ESM(am, ssm, getESMParams().get());
	} else if(!strcmp(sm_type, "aelm") || !strcmp(sm_type, "ael")){
		leven_marq = true;
		return new nt::AESM(am, ssm, getESMParams().get());
	} else if(!strcmp(sm_type, "iclm") || !strcmp(sm_type, "icl")){
		leven_marq = true;
		return new nt::ICLK(am, ssm, getICLKParams().get());
	} else if(!strcmp(sm_type, "fclm") || !strcmp(sm_type, "fcl")){
		leven_marq = true;
		return new nt::FCLK(am, ssm, getFCLKParams().get());
	} else if(!strcmp(sm_type, "falm") || !strcmp(sm_type, "fal")){
		leven_marq = true;
		return new nt::FALK(am, ssm, getFALKParams().get());
	} else if(!strcmp(sm_type, "ialm") || !strcmp(sm_type, "ial")){
		leven_marq = true;
		return new nt::IALK(am, ssm, getIALKParams().get());
	} else if(!strcmp(sm_type, "fcsd")){
		FCSDParams fcsd_params(max_iters, epsilon, sd_learning_rate,
			debug_mode, fc_hess_type);
		return new nt::FCSD(am, ssm, &fcsd_params);
	} else if(!strcmp(sm_type, "pf")){		
		return new nt::PF(am, ssm, getPFParams().get());
	} else if(!strcmp(sm_type, "pf100")){// PF with 100 particles
		pf_n_particles = 100;
		return new nt::PF(am, ssm, getPFParams().get());
	} else if(!strcmp(sm_type, "pf250")){// PF with 250 particles
		pf_n_particles = 250;
		return new nt::PF(am, ssm, getPFParams().get());
	} else if(!strcmp(sm_type, "pf500")){// PF with 500 particles
		pf_n_particles = 500;
		return new nt::PF(am, ssm, getPFParams().get());
	} else if(!strcmp(sm_type, "pf1k")){// PF with 1000 particles
		pf_n_particles = 1000;
		return new nt::PF(am, ssm, getPFParams().get());
	} else if(!strcmp(sm_type, "pf2k")){// PF with 2000 particles
		pf_n_particles = 2000;
		return new nt::PF(am, ssm, getPFParams().get());
	} else if(!strcmp(sm_type, "pf5k")){// PF with 5000 particles
		pf_n_particles = 5000;
		return new nt::PF(am, ssm, getPFParams().get());
	} else if(!strcmp(sm_type, "nn")){
		return new nt::NN(am, ssm, getNNParams().get());
	} else if(!strcmp(sm_type, "nn1k")){// NN with 1000 samples
		nn_n_samples = 1000;
		return new nt::NN(am, ssm, getNNParams().get());
	} else if(!strcmp(sm_type, "nn2k")){// NN with 2000 samples
		nn_n_samples = 2000;
		return new nt::NN(am, ssm, getNNParams().get());
	} else if(!strcmp(sm_type, "nn5k")){// NN with 5000 samples
		nn_n_samples = 5000;
		return new nt::NN(am, ssm, getNNParams().get());
	} else if(!strcmp(sm_type, "nn10k")){// NN with 10000 samples
		nn_n_samples = 10000;
		return new nt::NN(am, ssm, getNNParams().get());
	} else if(!strcmp(sm_type, "nn100k")){// NN with 100000 samples
		nn_n_samples = 100000;
		return new nt::NN(am, ssm, getNNParams().get());
	} else if(!strcmp(sm_type, "gnn")){// Graph based NN
		nn_index_type = 0;
		return new nt::NN(am, ssm, getNNParams().get());
	}
#ifndef DISABLE_REGNET
	else if(!strcmp(sm_type, "regnet")){
		return new nt::RegNet(am, ssm, getRegNetParams().get());
	}
#endif
	else{
		printf("Invalid NT search method provided: %s\n", sm_type);
		return nullptr;
	}
}
inline SSMEstParams_ getSSMEstParams(){
	return SSMEstParams_(new SSMEstimatorParams(static_cast<SSMEstimatorParams::EstType>(est_method),
		est_ransac_reproj_thresh, est_n_model_pts, est_refine, est_max_iters,
		est_max_subset_attempts, est_use_boost_rng, est_confidence, est_lm_max_iters));
}


inline SPIParamsType getSPIParams(){
	SPIParamsType spi_params;
	switch(static_cast<SPIType>(spi_type)){
	case SPIType::None:
		break;
	case SPIType::PixDiff:
		spi_params.push_back(spi_pix_diff_thresh);
		break;
	case SPIType::Gradient:
		spi_params.push_back(spi_grad_thresh);
		spi_params.push_back(spi_grad_use_union);
		break;
	case SPIType::GFTT:
		spi_params.push_back(spi_gftt_max_corners);
		spi_params.push_back(spi_gftt_quality_level);
		spi_params.push_back(spi_gftt_min_distance);
		spi_params.push_back(spi_gftt_block_size);
		spi_params.push_back(spi_gftt_use_harris_detector);
		spi_params.push_back(spi_gftt_k);
		spi_params.push_back(spi_gftt_use_union);
		spi_params.push_back(spi_gftt_neigh_offset);
		break;
	default:
		throw utils::InvalidArgument("Invalid SPI type provided");
	}
	return spi_params;
}
inline ESMParams_ getESMParams(){
	return 	ESMParams_(new ESMParams(max_iters, epsilon,
		static_cast<ESMParams::JacType>(esm_jac_type),
		static_cast<ESMParams::HessType>(esm_hess_type),
		sec_ord_hess, esm_chained_warp, leven_marq, lm_delta_init,
		lm_delta_update, enable_learning,
		static_cast<SPIType>(spi_type), getSPIParams(), debug_mode));
}
inline FCLKParams_ getFCLKParams(){
	return 	FCLKParams_(new FCLKParams(max_iters, epsilon,
		static_cast<FCLKParams::HessType>(fc_hess_type), sec_ord_hess,
		fc_chained_warp, leven_marq, lm_delta_init, lm_delta_update,
		enable_learning, fc_write_ssm_updates, fc_show_grid,
		fc_show_patch, fc_patch_resize_factor, fc_debug_mode));
}
inline ICLKParams_ getICLKParams(){
	return 	ICLKParams_(new ICLKParams(max_iters, epsilon,
		static_cast<ICLKParams::HessType>(ic_hess_type), sec_ord_hess,
		ic_update_ssm, ic_chained_warp, leven_marq, lm_delta_init,
		lm_delta_update, enable_learning, debug_mode));
}
inline FALKParams_ getFALKParams(){
	return FALKParams_(new FALKParams(max_iters, epsilon,
		static_cast<FALKParams::HessType>(fa_hess_type), sec_ord_hess,
		fa_show_grid, fa_show_patch, fa_patch_resize_factor,
		fa_write_frames, leven_marq, lm_delta_init,
		lm_delta_update, enable_learning, debug_mode));
}
inline IALKParams_ getIALKParams(){
	return IALKParams_(new IALKParams(max_iters, epsilon,
		static_cast<IALKParams::HessType>(ia_hess_type), sec_ord_hess,
		leven_marq, lm_delta_init, lm_delta_update, debug_mode));
}
//! params for PF SM
inline PFParams_ getPFParams(){
	vectorvd pf_ssm_sigma, pf_ssm_mean;
	getSamplerParams(pf_ssm_sigma, pf_ssm_mean, pf_ssm_sigma_ids, pf_ssm_mean_ids, "PF");
	printf("pf_ssm_sigma: \n");
	for(unsigned int i = 0; i < pf_ssm_sigma.size(); ++i){
		printf("%d: \n", i);
		for(unsigned int j = 0; j < pf_ssm_sigma[i].size(); ++j){
			printf("%f\t", pf_ssm_sigma[i][j]);
		}
		printf("\n");
	}
	return PFParams_(new PFParams(pf_max_iters, pf_n_particles, epsilon,
		static_cast<PFParams::DynamicModel>(pf_dynamic_model),
		static_cast<PFParams::UpdateType>(pf_update_type),
		static_cast<PFParams::LikelihoodFunc>(pf_likelihood_func),
		static_cast<PFParams::ResamplingType>(pf_resampling_type),
		static_cast<PFParams::MeanType>(pf_mean_type),
		pf_reset_to_mean, pf_ssm_sigma, pf_ssm_mean,
		pf_update_distr_wts, pf_min_distr_wt,
		pf_adaptive_resampling_thresh, pf_pix_sigma,
		pf_measurement_sigma, pf_show_particles,
		enable_learning, pf_jacobian_as_sigma, pf_debug_mode));
}
inline NNParams_ getNNParams(){
	string saved_index_dir = cv::format("log/NN/%s/%s",
		actor.c_str(), seq_name.c_str());
	if(nn_save_index && !boost::filesystem::exists(saved_index_dir)){
		printf("NN data directory: %s does not exist. Creating it...\n", saved_index_dir.c_str());
		boost::filesystem::create_directories(saved_index_dir);
	}
	vectorvd nn_ssm_sigma, nn_ssm_mean;
	getSamplerParams(nn_ssm_sigma, nn_ssm_mean, nn_ssm_sigma_ids, nn_ssm_mean_ids, "NN");

	gnn::GNNParams gnn_params(
		nn_gnn_degree,
		nn_gnn_max_steps,
		nn_gnn_cmpt_dist_thresh,
		nn_gnn_random_start,
		nn_gnn_verbose);

	return NNParams_(new NNParams(
		&gnn_params, nn_n_samples, nn_max_iters,
		epsilon, nn_ssm_sigma, nn_ssm_mean, nn_pix_sigma,
		nn_additive_update, nn_show_samples, nn_add_samples_gap,
		nn_n_samples_to_add, nn_remove_samples, nn_load_index, nn_save_index,
		saved_index_dir, debug_mode));
}

//! params for NN SM
#ifndef DISABLE_FLANN
inline FLANNParams_ getFLANNParams(){
	return FLANNParams_(new FLANNParams(
		static_cast<FLANNParams::SearchType>(nn_search_type),
		static_cast<FLANNParams::IdxType>(nn_index_type),
		static_cast<FLANNParams::IdxType>(nn_fgnn_index_type),
		nn_srch_checks,
		nn_srch_eps,
		nn_srch_sorted,
		nn_srch_max_neighbors,
		nn_srch_cores,
		nn_srch_matrices_in_gpu_ram,
		static_cast<flann::tri_type>(nn_srch_use_heap),
		nn_kdt_trees,
		nn_km_branching,
		nn_km_iterations,
		static_cast<flann::flann_centers_init_t>(nn_km_centers_init),
		nn_km_cb_index,
		nn_kdts_leaf_max_size,
		nn_kdtc_leaf_max_size,
		nn_hc_branching,
		static_cast<flann::flann_centers_init_t>(nn_hc_centers_init),
		nn_hc_trees,
		nn_hc_leaf_max_size,
		nn_auto_target_precision,
		nn_auto_build_weight,
		nn_auto_memory_weight,
		nn_auto_sample_fraction
		));
}
#endif

#ifndef DISABLE_FEAT
inline FLANNCVParams_ getFLANNCVParams(){
	return FLANNCVParams_(new FLANNCVParams(
		static_cast<FLANNCVParams::IdxType>(nn_index_type),
		nn_srch_checks,
		nn_srch_eps,
		nn_srch_sorted,
		nn_kdt_trees,
		nn_km_branching,
		nn_km_iterations,
		static_cast<cvflann::flann_centers_init_t>(nn_km_centers_init),
		nn_km_cb_index,
		nn_kdts_leaf_max_size,
		nn_kdtc_leaf_max_size,
		nn_hc_branching,
		static_cast<cvflann::flann_centers_init_t>(nn_hc_centers_init),
		nn_hc_trees,
		nn_hc_leaf_max_size,
		nn_auto_target_precision,
		nn_auto_build_weight,
		nn_auto_memory_weight,
		nn_auto_sample_fraction
		));
}
inline DetectorParamsType getDetectorParams(DetectorType detector_type){
	DetectorParamsType detector_params;
	switch(detector_type){
	case DetectorType::NONE:
		break;
	case DetectorType::SIFT:
		detector_params.push_back(sift_n_features);
		detector_params.push_back(sift_n_octave_layers);
		detector_params.push_back(sift_contrast_thresh);
		detector_params.push_back(sift_edge_thresh);
		detector_params.push_back(sift_sigma);
		break;
	case DetectorType::SURF:
		detector_params.push_back(surf_hessian_threshold);
		detector_params.push_back(surf_n_octaves);
		detector_params.push_back(surf_n_octave_layers);
		detector_params.push_back(surf_extended);
		detector_params.push_back(surf_upright);
		break;
	case DetectorType::FAST:
		detector_params.push_back(fast_threshold);
		detector_params.push_back(fast_non_max_suppression);
		detector_params.push_back(fast_type);
		break;
	case DetectorType::BRISK:
		detector_params.push_back(brisk_thresh);
		detector_params.push_back(brisk_octaves);
		detector_params.push_back(brisk_pattern_scale);
		break;
	case DetectorType::MSER:
		detector_params.push_back(mser_delta);
		detector_params.push_back(mser_min_area);
		detector_params.push_back(mser_max_area);
		detector_params.push_back(mser_max_variation);
		detector_params.push_back(mser_min_diversity);
		detector_params.push_back(mser_max_evolution);
		detector_params.push_back(mser_area_threshold);
		detector_params.push_back(mser_min_margin);
		detector_params.push_back(mser_edge_blur_size);
		break;
	case DetectorType::ORB:
		detector_params.push_back(orb_n_features);
		detector_params.push_back(orb_scale_factor);
		detector_params.push_back(orb_n_levels);
		detector_params.push_back(orb_edge_threshold);
		detector_params.push_back(orb_first_level);
		detector_params.push_back(orb_WTA_K);
		detector_params.push_back(orb_score_type);
		detector_params.push_back(orb_patch_size);
		detector_params.push_back(orb_fast_threshold);
		break;
	case DetectorType::AGAST:
		detector_params.push_back(agast_threshold);
		detector_params.push_back(agast_non_max_suppression);
		detector_params.push_back(agast_type);
		break;
	case DetectorType::GFTT:
		detector_params.push_back(gftt_max_corners);
		detector_params.push_back(gftt_quality_level);
		detector_params.push_back(gftt_min_distance);
		detector_params.push_back(gftt_block_size);
		detector_params.push_back(gftt_use_harris_detector);
		detector_params.push_back(gftt_k);
		break;
	case DetectorType::Star:
		detector_params.push_back(star_max_size);
		detector_params.push_back(star_response_threshold);
		detector_params.push_back(star_line_threshold_projected);
		detector_params.push_back(star_line_threshold_binarized);
		detector_params.push_back(star_suppress_nonmax_size);
		break;
	case DetectorType::MSD:
		detector_params.push_back(msd_patch_radius);
		detector_params.push_back(msd_search_area_radius);
		detector_params.push_back(msd_nms_radius);
		detector_params.push_back(msd_nms_scale_radius);
		detector_params.push_back(msd_th_saliency);
		detector_params.push_back(msd_kNN);
		detector_params.push_back(msd_scale_factor);
		detector_params.push_back(msd_n_scales);
		detector_params.push_back(msd_compute_orientation);
		break;
	default:
		throw utils::InvalidArgument("Invalid detector type provided");
	}
	return detector_params;
}
inline DescriptorParamsType getDescriptorParams(DescriptorType descriptor_type){
	DescriptorParamsType descriptor_params;
	switch(descriptor_type){
	case DescriptorType::SIFT:
		descriptor_params.push_back(sift_n_features);
		descriptor_params.push_back(sift_n_octave_layers);
		descriptor_params.push_back(sift_contrast_thresh);
		descriptor_params.push_back(sift_edge_thresh);
		descriptor_params.push_back(sift_sigma);
		break;
	case DescriptorType::SURF:
		descriptor_params.push_back(surf_hessian_threshold);
		descriptor_params.push_back(surf_n_octaves);
		descriptor_params.push_back(surf_n_octave_layers);
		descriptor_params.push_back(surf_extended);
		descriptor_params.push_back(surf_upright);
		break;
	case DescriptorType::BRISK:
		descriptor_params.push_back(brisk_thresh);
		descriptor_params.push_back(brisk_octaves);
		descriptor_params.push_back(brisk_pattern_scale);
		break;
	case DescriptorType::ORB:
		descriptor_params.push_back(orb_n_features);
		descriptor_params.push_back(orb_scale_factor);
		descriptor_params.push_back(orb_n_levels);
		descriptor_params.push_back(orb_edge_threshold);
		descriptor_params.push_back(orb_first_level);
		descriptor_params.push_back(orb_WTA_K);
		descriptor_params.push_back(orb_score_type);
		descriptor_params.push_back(orb_patch_size);
		descriptor_params.push_back(orb_fast_threshold);
		break;
	case DescriptorType::BRIEF:
		descriptor_params.push_back(brief_bytes);
		descriptor_params.push_back(brief_use_orientation);
		break;
	case DescriptorType::FREAK:
		descriptor_params.push_back(freak_orientation_normalized);
		descriptor_params.push_back(freak_scale_normalized);
		descriptor_params.push_back(freak_pattern_scale);
		descriptor_params.push_back(freak_n_octaves);
		break;
	case DescriptorType::LUCID:
		descriptor_params.push_back(lucid_kernel);
		descriptor_params.push_back(lucid_blur_kernel);
		break;
	case DescriptorType::LATCH:
		descriptor_params.push_back(latch_bytes);
		descriptor_params.push_back(latch_rotation_invariance);
		descriptor_params.push_back(latch_half_ssd_size);
		break;
	case DescriptorType::DAISY:
		descriptor_params.push_back(daisy_radius);
		descriptor_params.push_back(daisy_q_radius);
		descriptor_params.push_back(daisy_q_theta);
		descriptor_params.push_back(daisy_q_hist);
		descriptor_params.push_back(daisy_norm);
		descriptor_params.push_back(daisy_H);
		descriptor_params.push_back(daisy_interpolation);
		descriptor_params.push_back(daisy_use_orientation);
		break;
	case DescriptorType::VGG:
		descriptor_params.push_back(vgg_desc);
		descriptor_params.push_back(vgg_isigma);
		descriptor_params.push_back(vgg_img_normalize);
		descriptor_params.push_back(vgg_use_scale_orientation);
		descriptor_params.push_back(vgg_scale_factor);
		descriptor_params.push_back(vgg_dsc_normalize);
		break;
	case DescriptorType::BoostDesc:
		descriptor_params.push_back(boost_desc_desc);
		descriptor_params.push_back(boost_desc_use_scale_orientation);
		descriptor_params.push_back(boost_desc_scale_factor);
		break;
	default:
		throw utils::InvalidArgument("Invalid descriptor type provided");
	}
	return descriptor_params;
}

#endif

#ifndef DISABLE_REGNET
//! params for RegNet SM
inline RegNetParams_ getRegNetParams(){
	string saved_index_dir = cv::format("log/RegNet/%s/%s",
		actor.c_str(), seq_name.c_str());
	if(!boost::filesystem::exists(saved_index_dir)){
		printf("RegNet data directory: %s does not exist. Creating it...\n", saved_index_dir.c_str());
		boost::filesystem::create_directories(saved_index_dir);
	}
	vectorvd rg_ssm_sigma, rg_ssm_mean;
	getSamplerParams(rg_ssm_sigma, rg_ssm_mean, rg_ssm_sigma_ids, rg_ssm_mean_ids, "RegNet");
	return RegNetParams_(new RegNetParams(rg_max_iters, rg_n_samples, epsilon,
		rg_ssm_sigma, rg_ssm_mean, rg_pix_sigma,
		rg_additive_update, rg_show_samples, rg_add_points, rg_remove_points,
		rg_load_index, rg_save_index, saved_index_dir, debug_mode, rg_nepochs, rg_bs,
		rg_preproc, rg_solver, rg_train, rg_mean, rg_dbg, rg_pretrained));

}
#endif
//! multi layer non templated PF
inline bool getPFk(vector<TrackerBase*> &trackers, const char *am_type,
	const char *ssm_type, const char *ilm_type){
	if(pfk_ssm_sigma_ids.size() < static_cast<unsigned int>(pfk_n_layers)){
		printf("Insufficient sigma IDs specified for %d layer PF: %lu\n", pfk_n_layers, pfk_ssm_sigma_ids.size());
		return false;
	}
	//! take the last 'pfk_n_layers' ssm_sigma_ids added to pfk_ssm_sigma_ids so that the ones specified 
	//! in modules.cfg can be completely overridden at runtime by command line arguments that are parsed last
	unsigned int start_id = pfk_ssm_sigma_ids.size() - static_cast<unsigned int>(pfk_n_layers);
	for(unsigned int layer_id = start_id; layer_id < pfk_ssm_sigma_ids.size(); ++layer_id){
		pf_ssm_sigma_ids = pfk_ssm_sigma_ids[layer_id];
		trackers.push_back(getSM("pf", am_type, ssm_type, ilm_type));
		if(!trackers.back()){ return false; }
	}
	return true;
}
//! multi layer non templated NN
inline bool getNNk(vector<TrackerBase*> &trackers, const char *am_type,
	const char *ssm_type, const char *ilm_type){
	if(nnk_ssm_sigma_ids.size() < static_cast<unsigned int>(nnk_n_layers)){
		printf("Insufficient sigma IDs specified for %d layer NN: %lu\n", nnk_n_layers, nnk_ssm_sigma_ids.size());
		return false;
	}
	//! take the last 'nnk_n_layers' ssm_sigma_ids added to nnk_ssm_sigma_ids so that the ones specified 
	//! in modules.cfg can be completely overridden at runtime by command line arguments that are parsed last
	unsigned int start_id = nnk_ssm_sigma_ids.size() - static_cast<unsigned int>(nnk_n_layers);
	for(unsigned int layer_id = start_id; layer_id < nnk_ssm_sigma_ids.size(); ++layer_id){
		nn_ssm_sigma_ids = nnk_ssm_sigma_ids[layer_id];
		trackers.push_back(getSM("nn", am_type, ssm_type, ilm_type));
		if(!trackers.back()){ return false; }
	}
	return true;
}
//! Non templated composite search methods
inline TrackerBase *getCompositeSM(const char *sm_type,
	const char *am_type, const char *ssm_type, const char *ilm_type){
	//! check composite SMs first to avoid creating unnecessary instances of AM and SSM
	if(!strcmp(sm_type, "pfic")){// PF + ICLK
		vector<TrackerBase*> trackers;
		trackers.push_back(getSM("pf", am_type, ssm_type, ilm_type));
		trackers.push_back(getSM("ic", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pffc")){// PF + FCLK
		vector<TrackerBase*> trackers;
		trackers.push_back(getSM("pf", am_type, ssm_type, ilm_type));
		trackers.push_back(getSM("fc", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfes")){// PF + FCLK
		vector<TrackerBase*> trackers;
		trackers.push_back(getSM("pf", am_type, ssm_type, ilm_type));
		trackers.push_back(getSM("esm", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfk")){
		vector<TrackerBase*> trackers;
		if(!getPFk(trackers, am_type, ssm_type, ilm_type)){ return nullptr; }
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfkic")){// PFk + ICLK
		vector<TrackerBase*> trackers;
		trackers.resize(2);
		if(!getPFk(trackers, am_type, ssm_type, ilm_type)){ return nullptr; }
		trackers.push_back(getSM("ic", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfkfc")){// PFk + ICLK
		vector<TrackerBase*> trackers;
		if(!getPFk(trackers, am_type, ssm_type, ilm_type)){ return nullptr; }
		trackers.push_back(getSM("fc", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfkes")){// PFk + ESM
		vector<TrackerBase*> trackers;
		if(!getPFk(trackers, am_type, ssm_type, ilm_type)){ return nullptr; }
		trackers.push_back(getSM("esm", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "pfrk")){// PF + RKLT
		vector<TrackerBase*> trackers;
		trackers.push_back(getSM("pf", am_type, ssm_type, ilm_type));
		trackers.push_back(getCompositeSM("rkl", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnic")){// NN + ICLK
		vector<TrackerBase*> trackers;
		trackers.push_back(getSM("nn", am_type, ssm_type, ilm_type));
		trackers.push_back(getSM("ic", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnfc")){// NN + FCLK
		vector<TrackerBase*> trackers;
		trackers.push_back(getSM("nn", am_type, ssm_type, ilm_type));
		trackers.push_back(getSM("fc", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnk")){// Multiple layers of NN
		vector<TrackerBase*> trackers;
		if(!getNNk(trackers, am_type, ssm_type, ilm_type)){ return nullptr; }
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnkic")){// NNk + ICLK
		vector<TrackerBase*> trackers;
		trackers.resize(2);
		if(!getNNk(trackers, am_type, ssm_type, ilm_type)){ return nullptr; }
		trackers.push_back(getSM("ic", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnkfc")){// NNk + ICLK
		vector<TrackerBase*> trackers;
		if(!getNNk(trackers, am_type, ssm_type, ilm_type)){ return nullptr; }
		trackers.push_back(getSM("fc", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnkes")){// NNk + ESM
		vector<TrackerBase*> trackers;
		if(!getNNk(trackers, am_type, ssm_type, ilm_type)){ return nullptr; }
		trackers.push_back(getSM("esm", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "nnrk")){// NN + RKLT
		vector<TrackerBase*> trackers;
		trackers.push_back(getSM("nn", am_type, ssm_type, ilm_type));
		trackers.push_back(getCompositeSM("rkl", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "grid")){
		if(!strcmp(grid_sm, "flow")){
			GridTrackerFlowParams grid_params(
				grid_res, grid_res, grid_patch_size, grid_patch_size,
				grid_pyramid_levels, grid_use_const_grad, grid_min_eig_thresh,
				max_iters, epsilon, grid_show_trackers, debug_mode);
			resx = grid_params.getResX();
			resy = grid_params.getResY();
			return new nt::GridTrackerFlow(AM(getAM(am_type, ilm_type)), SSM(getSSM(ssm_type)),
				&grid_params, getSSMEstParams().get());
		} else{
			return nullptr;
		}
	}
#ifndef DISABLE_GRID
	else if(!strcmp(sm_type, "gric")){
		//! Grid + ICLK
		vector<TrackerBase*> trackers;
		trackers.push_back(getTracker("grid", am_type, ssm_type, ilm_type));
		trackers.push_back(getSM("ic", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "grfc")){
		//! Grid + FCLK
		vector<TrackerBase*> trackers;
		trackers.push_back(getTracker("grid", am_type, ssm_type, ilm_type));
		trackers.push_back(getSM("fc", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "gres")){
		//! Grid + ESM
		vector<TrackerBase*> trackers;
		trackers.push_back(getTracker("grid", am_type, ssm_type, ilm_type));
		trackers.push_back(getSM("fc", am_type, ssm_type, ilm_type));
		CascadeParams casc_params(casc_enable_feedback, casc_auto_reinit,
			casc_reinit_err_thresh, casc_reinit_frame_gap);
		return new CascadeTracker(trackers, &casc_params);
	} else if(!strcmp(sm_type, "rklt") || !strcmp(sm_type, "rkl")){// Grid + Template tracker with SPI
		GridBase *grid_tracker = dynamic_cast<GridBase*>(getTracker("grid", am_type, ssm_type, ilm_type));
		nt::SearchMethod *templ_tracker = getSM(rkl_sm, am_type, ssm_type, ilm_type);
		if(!templ_tracker){
			// invalid or third party tracker has been specified in 'rkl_sm'
			throw utils::InvalidArgument(cv::format("Search method provided: %s is not compatible with RKLT", rkl_sm));
		}
		RKLTParams rkl_params(rkl_enable_spi, rkl_enable_feedback,
			rkl_failure_detection, rkl_failure_thresh, debug_mode);
		return new nt::RKLT(&rkl_params, grid_tracker, templ_tracker);
	} else if(!strcmp(sm_type, "lmes")){
		// LMS + ESM
		est_method = 1;
		GridBase *grid_tracker = dynamic_cast<GridBase*>(getTracker("grid", am_type, ssm_type, ilm_type));
		nt::SearchMethod *templ_tracker = getSM("esm", am_type, ssm_type, ilm_type);
		if(!templ_tracker){
			// invalid or third party tracker has been specified in 'rkl_sm'
			throw utils::InvalidArgument(cv::format("Search method provided: %s is not compatible with RKLT", rkl_sm));
		}
		RKLTParams rkl_params(rkl_enable_spi, rkl_enable_feedback,
			rkl_failure_detection, rkl_failure_thresh, debug_mode);
		return new nt::RKLT(&rkl_params, grid_tracker, templ_tracker);
	}
#endif	
	else{
		return nullptr;
	}
}
//! some AMs can be used as pixel mappers by other AMs, e.g. SCV with NCC or SSIM
inline ImageBase *getPixMapper(const char *pix_mapper_type){
	if(!pix_mapper_type)
		return nullptr;
	AMParams_ params = getAMParams(pix_mapper_type, "0");
	if(!params){ return nullptr; }
	if(!strcmp(pix_mapper_type, "ssd")){
		return new SSD(cast_params(SSD));
	} else if(!strcmp(pix_mapper_type, "nssd")){
		return new NSSD(cast_params(NSSD));
	} else if(!strcmp(pix_mapper_type, "zncc")){
		return new ZNCC(cast_params(ZNCC));
	} else if(!strcmp(pix_mapper_type, "scv")){
		return new SCV(cast_params(SCV));
	} else if(!strcmp(pix_mapper_type, "lscv")){
		return new LSCV(cast_params(LSCV));
	} else if(!strcmp(pix_mapper_type, "lrscv") || !strcmp(pix_mapper_type, "lrsc")){
		return new LRSCV(cast_params(LRSCV));
	} else if(!strcmp(pix_mapper_type, "rscv")){
		return new RSCV(cast_params(RSCV));
	} else if(!strcmp(pix_mapper_type, "mczncc") || !strcmp(pix_mapper_type, "zncc3")){
		return new MCZNCC(cast_params(MCZNCC));
	} else if(!strcmp(pix_mapper_type, "mcscv") || !strcmp(pix_mapper_type, "scv3")){
		return new MCSCV(cast_params(MCSCV));
	} else if(!strcmp(pix_mapper_type, "mcrscv") || !strcmp(pix_mapper_type, "rscv3")){
		return new MCRSCV(cast_params(MCRSCV));
	} else{
		throw utils::InvalidArgument("getPixMapper::Invalid pixel mapper type provided");
	}
}
//! Third Party Trackers
inline TrackerBase *getTracker(const char *tracker_type){
#ifndef DISABLE_THIRD_PARTY_TRACKERS
	if(!strcmp(tracker_type, "dsst")){
		DSSTParams dsst_params(dsst_padding, dsst_sigma, dsst_scale_sigma, dsst_lambda,
			dsst_learning_rate, dsst_number_scales, dsst_scale_step, dsst_number_rots, dsst_rot_step,
			dsst_resize_factor, dsst_is_scaling, dsst_is_rotating, dsst_bin_size);
		return new DSST(&dsst_params);
	} else if(!strcmp(tracker_type, "rsst")){
		dsst_is_rotating = 1;
		return getTracker("dsst");
	} else if(!strcmp(tracker_type, "kcf")){
		KCFParams kcf_params(
			kcf_padding,
			kcf_lambda,
			kcf_output_sigma_factor,
			kcf_interp_factor,
			kcf_kernel_sigma,
			kcf_number_scales,
			kcf_scale_step,
			kcf_scale_model_max_area,
			kcf_scale_sigma_factor,
			kcf_scale_learning_rate,
			kcf_is_scaling,
			kcf_resize_factor
			);
		return new KCF(&kcf_params);
	} else if(!strcmp(tracker_type, "rct")){
		RCTParams rct_params(rct_min_n_rect, rct_max_n_rect, rct_n_feat,
			rct_rad_outer_pos, rct_rad_search_win, rct_learning_rate);
		return new RCT(&rct_params);
	} else if(!strcmp(tracker_type, "cmt")){
		CMTParams cmt_params(cmt_estimate_scale, cmt_estimate_rotation,
			cmt_feat_detector, cmt_desc_extractor, cmt_resize_factor);
		return new cmt::CMT(&cmt_params);
	} else if(!strcmp(tracker_type, "strk")){
		struck::StruckParams strk_params(strk_config_path);
		return new struck::Struck(&strk_params);
	} else if(!strcmp(tracker_type, "frg")){
		FRGParams frg_params(frg_n_bins, frg_search_margin,
			static_cast<FRGParams::HistComparisonMetric>(frg_hist_cmp_metric),
			frg_resize_factor, frg_show_window);
		return new FRG(&frg_params);
	}
#ifndef DISABLE_TLD
	else if(!strcmp(tracker_type, "tld")){
		TLDParams tld_params(tld_tracker_enabled, tld_detector_enabled,
			tld_learning_enabled, tld_alternating);
		return new tld::TLD(&tld_params);
	}
#endif
#ifndef DISABLE_MIL
	else if(!strcmp(tracker_type, "mil")){
		MILParams mil_params(
			mil_algorithm,
			mil_num_classifiers,
			mil_overlap,
			mil_search_factor,
			mil_pos_radius_train,
			mil_neg_num_train,
			mil_num_features
			);
		return new MIL(&mil_params);
	}
#endif
#ifndef DISABLE_PFSL3
	else if(!strcmp(tracker_type, "pfsl3")){
		PFSL3Params pfsl3_params(pfsl3_p_x, pfsl3_p_y,
			pfsl3_state_std, pfsl3_rot, pfsl3_ncc_std, pfsl3_pca_std, pfsl3_ar_p,
			pfsl3_n, pfsl3_n_c, pfsl3_n_iter, pfsl3_sampling, pfsl3_capture,
			pfsl3_mean_check, pfsl3_outlier_flag, pfsl3_len,
			pfsl3_init_size, pfsl3_update_period, pfsl3_ff,
			pfsl3_basis_thr, pfsl3_max_num_basis, pfsl3_max_num_used_basis,
			pfsl3_show_weights, pfsl3_show_templates, pfsl3_debug_mode);
		return new PFSL3(&pfsl3_params);
	}
#endif
#ifndef DISABLE_VISP
	else if(!strcmp(tracker_type, "vp") || !strcmp(tracker_type, "vptt")){
		ViSPParams::SMType vp_sm_type = ViSPParams::SMType::FCLK;
		if(!strcmp(vptt_sm, "fc") || !strcmp(vptt_sm, "fclk")){
			vp_sm_type = ViSPParams::SMType::FCLK;
		} else if(!strcmp(vptt_sm, "ic") || !strcmp(vptt_sm, "iclk")){
			vp_sm_type = ViSPParams::SMType::ICLK;
		} else if(!strcmp(vptt_sm, "fa") || !strcmp(vptt_sm, "falk")){
			vp_sm_type = ViSPParams::SMType::FALK;
		} else if(!strcmp(vptt_sm, "esm")){
			vp_sm_type = ViSPParams::SMType::ESM;
		}
		ViSPParams::AMType vp_am_type = ViSPParams::AMType::SSD;
		if(!strcmp(vptt_am, "ssd")){
			vp_am_type = ViSPParams::AMType::SSD;
		} else if(!strcmp(vptt_am, "zncc")){
			vp_am_type = ViSPParams::AMType::ZNCC;
		} else if(!strcmp(vptt_am, "mi")){
			vp_am_type = ViSPParams::AMType::MI;
		}

		ViSPParams::SSMType vp_ssm_type = ViSPParams::SSMType::Homography;
		if(!strcmp(vptt_ssm, "8")){
			vp_ssm_type = ViSPParams::SSMType::Homography;
		} else if(!strcmp(vptt_ssm, "l8") || !strcmp(vptt_ssm, "sl3")){
			vp_ssm_type = ViSPParams::SSMType::SL3;
		} else if(!strcmp(vptt_ssm, "6")){
			vp_ssm_type = ViSPParams::SSMType::Affine;
		} else if(!strcmp(vptt_ssm, "4")){
			vp_ssm_type = ViSPParams::SSMType::Similarity;
		} else if(!strcmp(vptt_ssm, "3")){
			vp_ssm_type = ViSPParams::SSMType::Isometry;
		} else if(!strcmp(vptt_ssm, "2")){
			vp_ssm_type = ViSPParams::SSMType::Translation;
		}

		ViSPParams visp_params(
			vp_sm_type, vp_am_type, vp_ssm_type,
			vptt_max_iters, vptt_res, vptt_res, vptt_lambda,
			vptt_thresh_grad, vptt_pyr_n_levels,
			vptt_pyr_level_to_stop
			);
		return new ViSP(&visp_params);
	}
#endif
#ifndef DISABLE_CV3
	else if(!strcmp(tracker_type, "cv3")){
		CV3::TrackerType _tracker_type;
		if(cv3_tracker_type == "mil" || cv3_tracker_type == "0"){
			_tracker_type = CV3::TrackerType::MIL;
		} else if(cv3_tracker_type == "boost" || cv3_tracker_type == "bst" || cv3_tracker_type == "1"){
			_tracker_type = CV3::TrackerType::BOOSTING;
		} else if(cv3_tracker_type == "mdf" || cv3_tracker_type == "2"){
			_tracker_type = CV3::TrackerType::MEDIANFLOW;
		} else if(cv3_tracker_type == "tld" || cv3_tracker_type == "3"){
			_tracker_type = CV3::TrackerType::TLD;
		} else if(cv3_tracker_type == "kcf" || cv3_tracker_type == "4"){
			_tracker_type = CV3::TrackerType::KCF;
		} else if(cv3_tracker_type == "gtrn" || cv3_tracker_type == "5"){
			_tracker_type = CV3::TrackerType::GOTURN;
		} else{
			throw utils::InvalidArgument(cv::format("Invalid tracker type provided: %s", cv3_tracker_type.c_str()));
		}
		CV3Params cv3_params(_tracker_type);
		return new CV3(&cv3_params);
	}
#endif

#ifndef DISABLE_DFT
	else if(!strcmp(tracker_type, "dft")){
		dft::DFTParams dft_params(dft_res_to_l, dft_p_to_l, dft_max_iter,
			dft_max_iter_single_level, dft_pyramid_smoothing_variance,
			dft_presmoothing_variance, dft_n_control_points_on_edge,
			dft_b_adaptative_choice_of_points, dft_b_normalize_descriptors,
			static_cast<OptimizationType>(dft_optimization_type));
		return new dft::DFT(&dft_params);
	}
#endif
#ifndef DISABLE_GOTURN
	else if(!strcmp(tracker_type, "goturn") || !strcmp(tracker_type, "gtrn")){
		GOTURNParams gtrn_params(gtrn_do_train, gtrn_gpu_id, gtrn_show_intermediate_output,
			gtrn_model_file.c_str(), gtrn_trained_file.c_str());
		return new GOTURN(&gtrn_params);
	}
#endif
#ifndef DISABLE_XVISION
	else if(strstr(tracker_type, "xv")){
		using_xv_tracker = true;
		XVParams &xv_params(xv_visualize, xv_steps_per_frame, false, false);
		if(!strcmp(tracker_type, "xv1r")){
			return new XVSSDRotate(&xv_params);
		} else if(!strcmp(tracker_type, "xv1s")){
			return new XVSSDScaling(&xv_params);
		} else if(!strcmp(tracker_type, "xv2")){
			return new XVSSDTrans(&xv_params);
		} else if(!strcmp(tracker_type, "xv3")){
			return new XVSSDRT(&xv_params);
		} else if(!strcmp(tracker_type, "xv4")){
			return new XVSSDSE2(&xv_params);
		} else if(!strcmp(tracker_type, "xv6")){
			return new XVSSDAffine(&xv_params);
		} else if(!strcmp(tracker_type, "xv1p")){
			return new XVSSDPyramidRotate(&xv_params, xv_no_of_levels, xv_scale);
		} else if(!strcmp(tracker_type, "xv2p")){
			return new XVSSDPyramidTrans(&xv_params, xv_no_of_levels, xv_scale);
		} else if(!strcmp(tracker_type, "xv3p")){
			return new XVSSDPyramidRT(&xv_params, xv_no_of_levels, xv_scale);
		} else if(!strcmp(tracker_type, "xv4p")){
			return new XVSSDPyramidSE2(&xv_params, xv_no_of_levels, xv_scale);
		} else if(!strcmp(tracker_type, "xv6p")){
			return new XVSSDPyramidAffine(&xv_params, xv_no_of_levels, xv_scale);
		} else if(!strcmp(tracker_type, "xve")){
			return new XVEdgeTracker(&xv_params);
		} else if(!strcmp(tracker_type, "xvc")){
			return new XVColor(&xv_params);
		} else if(!strcmp(tracker_type, "xvg")){
			return new XVSSDGrid(&xv_params,
				xv_tracker_type, xvg_grid_size_x, xvg_grid_size_y, xv_patch_size,
				xvg_reset_pos, reset_template, xvg_sel_reset_thresh, xvg_reset_wts,
				xvg_adjust_lines, xvg_update_wts, debug_mode);
		} else if(!strcmp(tracker_type, "xvgl")){
			return new XVSSDGridLine(&xv_params,
				xv_tracker_type, xvg_grid_size_x, xvg_grid_size_y,
				xv_patch_size, xvg_use_constant_slope, xvg_use_ls, xvg_inter_alpha_thresh,
				xvg_intra_alpha_thresh, xvg_reset_pos, reset_template, debug_mode);
		} else{
			stringstream err_msg;
			err_msg << "Invalid Xvision tracker type provided : " << tracker_type << "\n";
			throw utils::InvalidArgument(err_msg.str());
		}
	}
#endif
#endif
	return nullptr;
}

_MTF_END_NAMESPACE

#endif
