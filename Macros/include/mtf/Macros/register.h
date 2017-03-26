#ifndef MTF_REGISTER_H
#define MTF_REGISTER_H

//! include statements and macros for registering different combinations of the various modules

//! Appearance Models
#include "mtf/AM/SSD.h"
#include "mtf/AM//NSSD.h"
#include "mtf/AM//ZNCC.h"
#include "mtf/AM//SCV.h"
#include "mtf/AM//LSCV.h"
#include "mtf/AM//RSCV.h"
#include "mtf/AM//LRSCV.h"
#include "mtf/AM//KLD.h"
#include "mtf/AM//LKLD.h"	
#include "mtf/AM//MI.h"
#include "mtf/AM//SPSS.h"
#include "mtf/AM//SSIM.h"
#include "mtf/AM//NCC.h"
#include "mtf/AM//CCRE.h"
#include "mtf/AM//RIU.h"
#include "mtf/AM//NGF.h"
#include "mtf/AM//SAD.h"
//! Multi Channel Variants 
#include "mtf/AM//MCSSD.h"
#include "mtf/AM//MCNCC.h"
#include "mtf/AM//MCMI.h"
#include "mtf/AM/MCSCV.h"
#include "mtf/AM/MCLSCV.h"
#include "mtf/AM/MCRSCV.h"
#include "mtf/AM/MCZNCC.h"
#include "mtf/AM/MCCCRE.h"
#include "mtf/AM/MCSSIM.h"
#include "mtf/AM/MCSPSS.h"
#include "mtf/AM/MCRIU.h"
#include "mtf/AM/MCSAD.h"
#ifndef DISABLE_DFM
#include "mtf/AM/DFM.h"
#endif
#ifndef DISABLE_PCA
#include "mtf/AM/PCA.h"
#include "mtf/AM/MCPCA.h"
#endif

//! State Space Models
#include "mtf/SSM/Homography.h"
#include "mtf/SSM/Affine.h"
#include "mtf/SSM/Similitude.h"
#include "mtf/SSM/Isometry.h"
#include "mtf/SSM/IST.h"
#include "mtf/SSM/Translation.h"
#include "mtf/SSM/LieHomography.h"
#include "mtf/SSM/CornerHomography.h"
#include "mtf/SSM/SL3.h"
#include "mtf/SSM/Spline.h"	

#ifndef DISABLE_DFM
#define _REGISTER_TRACKERS_DFM(SM, SSM) \
	template class mtf::SM< mtf::DFM, mtf::SSM >;
#define _REGISTER_AM_TRACKERS_DFM(SM) \
	template class mtf::SM< mtf::DFM >;
#else
#define _REGISTER_TRACKERS_DFM(SM, SSM)
#define _REGISTER_AM_TRACKERS_DFM(SM)
#endif

#ifndef DISABLE_PCA
#define _REGISTER_TRACKERS_PCA(SM, SSM) \
	template class mtf::SM< mtf::PCA, mtf::SSM >;\
	template class mtf::SM< mtf::MCPCA, mtf::SSM >;

#define _REGISTER_AM_TRACKERS_PCA(SM) \
	template class mtf::SM< mtf::PCA >;\
	template class mtf::SM< mtf::MCPCA >;

#else
#define _REGISTER_TRACKERS_PCA(SM, SSM)
#define _REGISTER_AM_TRACKERS_PCA(SM)
#endif

#define _REGISTER_TRACKERS_AM(SM, SSM) \
	template class mtf::SM< mtf::SSD,  mtf::SSM >;\
	template class mtf::SM< mtf::NSSD,  mtf::SSM >;\
	template class mtf::SM< mtf::ZNCC,  mtf::SSM >;\
	template class mtf::SM< mtf::SCV,  mtf::SSM >;\
	template class mtf::SM< mtf::RSCV,  mtf::SSM >;\
	template class mtf::SM< mtf::LSCV,  mtf::SSM >;\
	template class mtf::SM< mtf::LRSCV,  mtf::SSM >;\
	template class mtf::SM< mtf::KLD,  mtf::SSM >;\
	template class mtf::SM< mtf::LKLD,  mtf::SSM >;\
	template class mtf::SM< mtf::SPSS,  mtf::SSM >;\
	template class mtf::SM< mtf::SSIM,  mtf::SSM >;\
	template class mtf::SM< mtf::NCC,  mtf::SSM >;\
	template class mtf::SM< mtf::CCRE,  mtf::SSM >;\
	template class mtf::SM< mtf::MI,  mtf::SSM >;\
	template class mtf::SM< mtf::SAD,  mtf::SSM >;\
	template class mtf::SM< mtf::RIU,  mtf::SSM >;\
	template class mtf::SM< mtf::NGF,  mtf::SSM >;\
	template class mtf::SM< mtf::MCSSD,  mtf::SSM >;\
	template class mtf::SM< mtf::MCSCV,  mtf::SSM >;\
	template class mtf::SM< mtf::MCLSCV,  mtf::SSM >;\
	template class mtf::SM< mtf::MCRSCV,  mtf::SSM >;\
	template class mtf::SM< mtf::MCZNCC,  mtf::SSM >;\
	template class mtf::SM< mtf::MCNCC,  mtf::SSM >;\
	template class mtf::SM< mtf::MCMI,  mtf::SSM >;\
	template class mtf::SM< mtf::MCSSIM,  mtf::SSM >;\
	template class mtf::SM< mtf::MCSPSS,  mtf::SSM >;\
	template class mtf::SM< mtf::MCRIU,  mtf::SSM >;\
	template class mtf::SM< mtf::MCSAD,  mtf::SSM >;\
	template class mtf::SM< mtf::MCCCRE,  mtf::SSM >;\
	 _REGISTER_TRACKERS_DFM(SM, SSM)\
	 _REGISTER_TRACKERS_PCA(SM, SSM)


#define _REGISTER_TRACKERS(SM) \
	_REGISTER_TRACKERS_AM(SM, LieHomography)\
	_REGISTER_TRACKERS_AM(SM, CornerHomography)\
	_REGISTER_TRACKERS_AM(SM, SL3)\
	_REGISTER_TRACKERS_AM(SM, Homography)\
	_REGISTER_TRACKERS_AM(SM, Affine)\
	_REGISTER_TRACKERS_AM(SM, Similitude)\
	_REGISTER_TRACKERS_AM(SM, Isometry)\
	_REGISTER_TRACKERS_AM(SM, IST)\
	_REGISTER_TRACKERS_AM(SM, Translation)\
	_REGISTER_TRACKERS_AM(SM, Spline)\

#define _REGISTER_TRACKERS_SSM(SM) \
	template class mtf::SM< mtf::LieHomography>;\
	template class mtf::SM< mtf::CornerHomography>;\
	template class mtf::SM< mtf::SL3>;\
	template class mtf::SM< mtf::Homography>;\
	template class mtf::SM< mtf::Affine>;\
	template class mtf::SM< mtf::Similitude>;\
	template class mtf::SM< mtf::Isometry>;\
	template class mtf::SM< mtf::IST>;\
	template class mtf::SM< mtf::Translation>;\
	template class mtf::SM< mtf::Spline>;

#define _REGISTER_AM_TRACKERS(SM) \
	template class mtf::SM< mtf::SSD >;\
	template class mtf::SM< mtf::NSSD >;\
	template class mtf::SM< mtf::ZNCC >;\
	template class mtf::SM< mtf::SCV >;\
	template class mtf::SM< mtf::RSCV >;\
	template class mtf::SM< mtf::LSCV >;\
	template class mtf::SM< mtf::LRSCV >;\
	template class mtf::SM< mtf::KLD >;\
	template class mtf::SM< mtf::LKLD >;\
	template class mtf::SM< mtf::SPSS >;\
	template class mtf::SM< mtf::SSIM >;\
	template class mtf::SM< mtf::NCC >;\
	template class mtf::SM< mtf::CCRE >;\
	template class mtf::SM< mtf::MI >;\
	template class mtf::SM< mtf::RIU >;\
	template class mtf::SM< mtf::NGF >;\
	template class mtf::SM< mtf::SAD >;\
	template class mtf::SM< mtf::MCSSD >;\
	template class mtf::SM< mtf::MCSCV >;\
	template class mtf::SM< mtf::MCLSCV >;\
	template class mtf::SM< mtf::MCRSCV >;\
	template class mtf::SM< mtf::MCZNCC >;\
	template class mtf::SM< mtf::MCNCC >;\
	template class mtf::SM< mtf::MCMI >;\
	template class mtf::SM< mtf::MCSSIM >;\
	template class mtf::SM< mtf::MCSPSS >;\
	template class mtf::SM< mtf::MCRIU >;\
	template class mtf::SM< mtf::MCSAD >;\
	template class mtf::SM< mtf::MCCCRE >;\
	 _REGISTER_AM_TRACKERS_PCA(SM)\
	 _REGISTER_AM_TRACKERS_DFM(SM)

//registration macros for hierarchical trackers with 2 SSMs
#define _REGISTER_HTRACKERS_AM(SM, SSM, SSM2) \
	template class mtf::SM< mtf::SSD,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::NSSD,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::ZNCC,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::SCV,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::RSCV,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::LSCV,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::LRSCV,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::KLD,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::LKLD,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::SPSS,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::SSIM,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::NCC,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::CCRE,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::RIU,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::NGF,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::SAD,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCSSD,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCSCV,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCLSCV,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCRSCV,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCZNCC,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCNCC,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCMI,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCSSIM,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCSPSS,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCRIU,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCCCRE,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MCSAD,  mtf::SSM,  mtf::SSM2 >;\
	template class mtf::SM< mtf::MI,  mtf::SSM,  mtf::SSM2 >;

#define _REGISTER_HTRACKERS_SSM(SM, SSM) \
	_REGISTER_HTRACKERS_AM(SM, SSM, LieHomography)\
	_REGISTER_HTRACKERS_AM(SM, SSM, CornerHomography)\
	_REGISTER_HTRACKERS_AM(SM, SSM, SL3)\
	_REGISTER_HTRACKERS_AM(SM, SSM, Homography)\
	_REGISTER_HTRACKERS_AM(SM, SSM, Affine)\
	_REGISTER_HTRACKERS_AM(SM, SSM, Similitude)\
	_REGISTER_HTRACKERS_AM(SM, SSM, Isometry)\
	_REGISTER_HTRACKERS_AM(SM, SSM, IST)\
	_REGISTER_HTRACKERS_AM(SM, SSM, Translation)\
	_REGISTER_HTRACKERS_AM(SM, SSM, Spline)\

#define _REGISTER_HTRACKERS(SM) \
	_REGISTER_HTRACKERS_SSM(SM, LieHomography)\
	_REGISTER_HTRACKERS_SSM(SM, CornerHomography)\
	_REGISTER_HTRACKERS_SSM(SM, SL3)\
	_REGISTER_HTRACKERS_SSM(SM, Homography)\
	_REGISTER_HTRACKERS_SSM(SM, Affine)\
	_REGISTER_HTRACKERS_SSM(SM, Similitude)\
	_REGISTER_HTRACKERS_SSM(SM, Isometry)\
	_REGISTER_HTRACKERS_SSM(SM, IST)\
	_REGISTER_HTRACKERS_SSM(SM, Translation)\
	_REGISTER_HTRACKERS_SSM(SM, Spline)\

#define _REGISTER_ESM_AM(HT, JT, SSM) \
	template class mtf::FESM< mtf::SSD,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::NSSD,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::ZNCC,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::SCV,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::RSCV,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::LSCV,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::LRSCV,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::KLD,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::LKLD,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::SPSS,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::SSIM,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::NCC,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::CCRE,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::RIU,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::NGF,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::SAD,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCSSD,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCSCV,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCLSCV,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCRSCV,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCZNCC,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCNCC,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCMI,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCSSIM,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCSPSS,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCRIU,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCCCRE,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MCSAD,  mtf::SSM, mtf::HT, mtf::JT >;\
	template class mtf::FESM< mtf::MI,  mtf::SSM, mtf::HT, mtf::JT >;

#define _REGISTER_ESM(HT, JT) \
	_REGISTER_ESM_AM(HT, JT, LieHomography)\
	_REGISTER_ESM_AM(HT, JT, CornerHomography)\
	_REGISTER_ESM_AM(HT, JT, SL3)\
	_REGISTER_ESM_AM(HT, JT, Homography)\
	_REGISTER_ESM_AM(HT, JT, Affine)\
	_REGISTER_ESM_AM(HT, JT, Similitude)\
	_REGISTER_ESM_AM(HT, JT, Isometry)\
	_REGISTER_ESM_AM(HT, JT, IST)\
	_REGISTER_ESM_AM(HT, JT, Translation)\
	_REGISTER_ESM_AM(HT, JT, Spline)\

#endif



