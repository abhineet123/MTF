#ifndef MTF_TEST_H
#define MTF_TEST_H

#define DIAGNOSTICS_MODE

// parameters for the different modules
#include "mtf/Config/parameters.h"

//diagnostics for testing and debugging
#include "Diagnostics.h"

// appearance models
#include "mtf/AM/SSD.h"
#include "mtf/AM/NSSD.h"
#include "mtf/AM/ZNCC.h"
#include "mtf/AM/SCV.h"
#include "mtf/AM/LSCV.h"
#include "mtf/AM/RSCV.h"
#include "mtf/AM/LRSCV.h"
#include "mtf/AM/KLD.h"
#include "mtf/AM/LKLD.h"	
#include "mtf/AM/MI.h"
#include "mtf/AM/SPSS.h"
#include "mtf/AM/SSIM.h"
#include "mtf/AM/NCC.h"
#include "mtf/AM/CCRE.h"
#ifndef DISABLE_FMAPS
#include "mtf/AM//FMaps.h"
#endif
#ifndef DISABLE_PCA
#include "mtf/AM/PCA.h"
#endif

// state space models
#include "mtf/SSM/LieHomography.h"
#include "mtf/SSM/CornerHomography.h"
#include "mtf/SSM/SL3.h"
#include "mtf/SSM/Homography.h"
#include "mtf/SSM/Affine.h"
#include "mtf/SSM/Similitude.h"
#include "mtf/SSM/Isometry.h"
#include "mtf/SSM/Transcaling.h"
#include "mtf/SSM/Translation.h"


_MTF_BEGIN_NAMESPACE

using namespace params;

inline ImageBase *getPixMapperObj(const char *pix_mapper_type, ImgParams *img_params){
	if(!pix_mapper_type)
		return nullptr;
	if(!strcmp(pix_mapper_type, "ssd")){
		return new SSD(img_params);
	} else if(!strcmp(pix_mapper_type, "nssd")){
		NSSDParams *nssd_params = new NSSDParams(img_params, nssd_norm_pix_max, nssd_norm_pix_min, debug_mode);
		return new NSSD(nssd_params);
	} else if(!strcmp(pix_mapper_type, "zncc")){
		return new ZNCC(img_params);
	} else if(!strcmp(pix_mapper_type, "scv")){
		SCVParams *scv_params = new SCVParams(img_params, scv_use_bspl, scv_n_bins, scv_preseed, scv_pou,
			scv_weighted_mapping, scv_mapped_gradient, debug_mode);
		return new SCV(scv_params);
	} else if(!strcmp(pix_mapper_type, "lscv")){
		LSCVParams *lscv_params = new LSCVParams(img_params, lscv_sub_regions, lscv_sub_regions,
			lscv_spacing, lscv_spacing, scv_affine_mapping, scv_once_per_frame, scv_n_bins, scv_preseed,
			scv_weighted_mapping, lscv_show_subregions, debug_mode);
		return new LSCV(lscv_params);
	} else if(!strcmp(pix_mapper_type, "lrscv") || !strcmp(pix_mapper_type, "lrsc")){
		LRSCVParams *lrscv_params = new LRSCVParams(img_params, lscv_sub_regions, lscv_sub_regions,
			lscv_spacing, lscv_spacing, scv_affine_mapping, scv_once_per_frame, scv_n_bins, scv_preseed,
			scv_weighted_mapping, lscv_show_subregions, debug_mode);
		return new LRSCV(lrscv_params);
	} else if(!strcmp(pix_mapper_type, "rscv")){
		RSCVParams *rscv_params = new RSCVParams(img_params, scv_use_bspl, scv_n_bins, scv_preseed, scv_pou,
			scv_weighted_mapping, scv_mapped_gradient, debug_mode);
		return new RSCV(rscv_params);
	} else{
		throw std::invalid_argument("getPixMapperObj::Invalid pixel mapper type provided");
	}
}

template< class AMType >
DiagBase *getDiagnosticsObj(const char *ssm_type,
	typename AMType::ParamType *am_params = nullptr){
	DiagnosticsParams *diag_params = new DiagnosticsParams(
		static_cast<DiagnosticsParams::UpdateType>(diag_update),
		diag_show_corners, diag_show_patches,
		diag_enable_validation, diag_validation_prec);
	if(!strcmp(ssm_type, "lhom") || !strcmp(ssm_type, "l8")){
		LieHomographyParams lhomm_params(lhom_normalized_init, debug_mode);
		return new Diagnostics<AMType, LieHomography>(diag_params, am_params, &lhomm_params);
	} else if(!strcmp(ssm_type, "sl3")){
		SL3Params sl3_params(sl3_normalized_init, sl3_iterative_sample_mean,
			sl3_sample_mean_max_iters, sl3_sample_mean_eps, debug_mode);
		return new Diagnostics<AMType, mtf::SL3>(diag_params, am_params, &sl3_params);
	} else if(!strcmp(ssm_type, "hom") || !strcmp(ssm_type, "8")){
		HomographyParams hom_params(hom_normalized_init, hom_corner_based_sampling, debug_mode);
		return new Diagnostics<AMType, mtf::Homography>(diag_params, am_params, &hom_params);
	} else if(!strcmp(ssm_type, "chom") || !strcmp(ssm_type, "c8")){
		CornerHomographyParams chom_params(chom_normalized_init, chom_grad_eps, debug_mode);
		return new Diagnostics<AMType, mtf::CornerHomography>(diag_params, am_params, &chom_params);
	} else if(!strcmp(ssm_type, "aff") || !strcmp(ssm_type, "6")){
		AffineParams aff_params(aff_normalized_init, debug_mode);
		return new Diagnostics<AMType, mtf::Affine>(diag_params, am_params, &aff_params);
	} else if(!strcmp(ssm_type, "sim") || !strcmp(ssm_type, "4")){
		return new Diagnostics<AMType, mtf::Similitude>(diag_params, am_params);
	} else if(!strcmp(ssm_type, "iso") || !strcmp(ssm_type, "3")){
		return new Diagnostics<AMType, mtf::Isometry>(diag_params, am_params);
	} else if(!strcmp(ssm_type, "trs") || !strcmp(ssm_type, "3s")){
		return new Diagnostics<AMType, mtf::Transcaling>(diag_params, am_params);
	} else if(!strcmp(ssm_type, "tra") || !strcmp(ssm_type, "2")){
		return new Diagnostics<AMType, mtf::Translation>(diag_params, am_params);
	} else{
		printf("Invalid state space model provided: %s\n", ssm_type);
		return nullptr;
	}
}

inline DiagBase *getDiagnosticsObj(const char *am_type, const char *ssm_type){
	mtf::ImgParams img_params(resx, resy);
	if(!strcmp(am_type, "ssd")){
		return getDiagnosticsObj<SSD>(ssm_type, &img_params);
	} else if(!strcmp(am_type, "nssd")){
		NSSDParams *nssd_params = new NSSDParams(&img_params, nssd_norm_pix_max, nssd_norm_pix_min, debug_mode);
		return getDiagnosticsObj<NSSD>(ssm_type, nssd_params);
	} else if(!strcmp(am_type, "zncc")){
		return getDiagnosticsObj<ZNCC>(ssm_type, &img_params);
	} else if(!strcmp(am_type, "scv")){
		SCVParams *scv_params = new SCVParams(&img_params, scv_use_bspl, scv_n_bins, scv_preseed, scv_pou,
			scv_weighted_mapping, scv_mapped_gradient, debug_mode);
		return getDiagnosticsObj<SCV>(ssm_type, scv_params);
	} else if(!strcmp(am_type, "lscv")){
		LSCVParams *lscv_params = new LSCVParams(&img_params, lscv_sub_regions, lscv_sub_regions,
			lscv_spacing, lscv_spacing, scv_affine_mapping, scv_once_per_frame, scv_n_bins, scv_preseed,
			scv_weighted_mapping, lscv_show_subregions, debug_mode);
		return getDiagnosticsObj<LSCV>(ssm_type, lscv_params);
	} else if(!strcmp(am_type, "rscv")){
		RSCVParams *rscv_params = new RSCVParams(&img_params, scv_use_bspl, scv_n_bins, scv_preseed, scv_pou,
			scv_weighted_mapping, scv_mapped_gradient, debug_mode);
		return getDiagnosticsObj<RSCV>(ssm_type, rscv_params);
	} else if(!strcmp(am_type, "lrscv") || !strcmp(am_type, "lrsc")){
		LRSCVParams *lscv_params = new LRSCVParams(&img_params, lscv_sub_regions, lscv_sub_regions,
			lscv_spacing, lscv_spacing, scv_affine_mapping, scv_once_per_frame, scv_n_bins, scv_preseed,
			scv_weighted_mapping, lscv_show_subregions, debug_mode);
		return getDiagnosticsObj<LRSCV>(ssm_type, lscv_params);
	} else if(!strcmp(am_type, "kld")){
		KLDParams *kld_params = new KLDParams(&img_params, mi_n_bins, mi_pre_seed, mi_pou, debug_mode);
		return getDiagnosticsObj<KLD>(ssm_type, kld_params);
	} else if(!strcmp(am_type, "lkld")){
		LKLDParams *lkld_params = new LKLDParams(&img_params, lkld_sub_regions, lkld_sub_regions,
			lkld_spacing, lkld_spacing, lkld_n_bins, lkld_pre_seed, lkld_pou, debug_mode);
		return getDiagnosticsObj<LKLD>(ssm_type, lkld_params);
	} else if(!strcmp(am_type, "mi")){
		MIParams *mi_params = new MIParams(&img_params, mi_n_bins, mi_pre_seed, mi_pou,
			getPixMapperObj(pix_mapper, &img_params), debug_mode);
		return getDiagnosticsObj<MI>(ssm_type, mi_params);
	} else if(!strcmp(am_type, "spss")){
		SPSSParams *spss_params = new SPSSParams(&img_params, ssim_k1,
			getPixMapperObj(pix_mapper, &img_params));
		return getDiagnosticsObj<SPSS>(ssm_type, spss_params);
	} else if(!strcmp(am_type, "ssim")){
		SSIMParams *ssim_params = new SSIMParams(&img_params, ssim_k1, ssim_k2);
		return getDiagnosticsObj<SSIM>(ssm_type, ssim_params);
	} else if(!strcmp(am_type, "ncc")){
		NCCParams *ncc_params = new NCCParams(&img_params, ncc_fast_hess);
		return getDiagnosticsObj<NCC>(ssm_type, ncc_params);
	} else if(!strcmp(am_type, "ccre")){
		CCREParams *ccre_params = new CCREParams(&img_params, ccre_n_bins, ccre_pou, ccre_pre_seed,
			ccre_symmetrical_grad, ccre_n_blocks, debug_mode);
		return getDiagnosticsObj<CCRE>(ssm_type, ccre_params);
	}
#ifndef DISABLE_FMAPS
	else if(!strcmp(am_type, "fmaps")){
		FMapsParams fmaps_params(&img_params, fmaps_nfmaps, fmaps_layer_name, fmaps_vis, fmaps_zncc);
		return getDiagnosticsObj<FMaps>(ssm_type, &fmaps_params);
	}
#endif	
#ifndef DISABLE_PCA
	else if(!strcmp(am_type, "pca")){
		PCAParams pca_params(&img_params, db_root_path, actor,
			source_name, extraction_id, pca_n_feat, pca_len_history);
		return getDiagnosticsObj<PCA>(ssm_type, &pca_params);
	}
#endif
	else{
		printf("Invalid appearance model provided: %s\n", am_type);
		return nullptr;
	}
}
_MTF_END_NAMESPACE

#endif
