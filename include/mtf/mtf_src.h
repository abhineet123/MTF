#ifndef MTF_SRC_H
#define MTF_SRC_H

//! includes all sources for a no-library header only version of MTF

#ifndef ENABLE_ONLY_NT
//! search methods
#include "../SM/src/ESM.cc"
#include "../SM/src/ICLK.cc"
#include "../SM/src/FCLK.cc"
#include "../SM/src/FALK.cc"
#include "../SM/src/IALK.cc"
#include "../SM/src/PF.cc"
#ifndef DISABLE_FLANN
#include "../SM/src/NN.cc"
#include "../SM/src/FGNN.cc"
#endif
#include "../SM/src/GNN.cc"
//! composite search methods
#include "../SM/src/RKLT.cc"
#include "../SM/src/CascadeSM.cc"
#include "../SM/src/ParallelSM.cc"
#include "../SM/src/PyramidalSM.cc"
#endif
#ifndef DISABLE_GRID
#include "../SM/src/GridTracker.cc"
#include "../SM/src/GridTrackerCV.cc"
#ifndef ENABLE_ONLY_NT
#include "../SM/src/GridTrackerFlow.cc"
#endif
#ifndef DISABLE_FEAT
#ifndef DISABLE_FLANN
#include "../SM/src/FeatureTracker.cc"
#endif
#endif
#endif
#include "../SM/src/CascadeTracker.cc"
#include "../SM/src/ParallelTracker.cc"
#include "../SM/src/PyramidalTracker.cc"
#include "../SM/src/LineTracker.cc"
//! Non templated implementations of search methods
#include "../SM/src/NT/FCLK.cc"
#include "../SM/src/NT/ICLK.cc"
#include "../SM/src/NT/FALK.cc"
#include "../SM/src/NT/IALK.cc"
#include "../SM/src/NT/ESM.cc"
#include "../SM/src/NT/AESM.cc"
#include "../SM/src/NT/PF.cc"
#include "../SM/src/NT/NN.cc"

#include "../SM/src/NT/FCSD.cc"
#include "../SM/src/NT/GridTrackerFlow.cc"
#include "../SM/src/NT/RKLT.cc"
#ifndef DISABLE_REGNET
#include "../SM/src/NT/RegNet.cc"
#endif

// Obsolete/failed search methods
//#include "../SM/src/HACLK.cc"
//#include "../SM/src/ESMH.cc"
//#include "../SM/src/HESM.cc"
//#include "../SM/src/FESM.cc"
//#include "../SM/src/IALK2.cc"

//! appearance models
#include "../AM/src/SSDBase.cc"
#include "../AM/src/ImageBase.cc"
#include "../AM/src/SSD.cc"
#include "../AM/src/NSSD.cc"
#include "../AM/src/ZNCC.cc"
#include "../AM/src/SCV.cc"
#include "../AM/src/LSCV.cc"
#include "../AM/src/RSCV.cc"
#include "../AM/src/LRSCV.cc"
#include "../AM/src/KLD.cc"
#include "../AM/src/LKLD.cc"	
#include "../AM/src/MI.cc"
#include "../AM/src/SPSS.cc"
#include "../AM/src/SSIM.cc"
#include "../AM/src/NCC.cc"
#include "../AM/src/CCRE.cc"
#include "../AM/src/RIU.cc"
#include "../AM/src/NGF.cc"
#include "../AM/src/SAD.cc"
//! multi channel variants
#include "../AM/src/MCSSD.cc"
#include "../AM/src/MCSCV.cc"
#include "../AM/src/MCLSCV.cc"
#include "../AM/src/MCRSCV.cc"
#include "../AM/src/MCZNCC.cc"
#include "../AM/src/MCNCC.cc"
#include "../AM/src/MCMI.cc"
#include "../AM/src/MCSSIM.cc"
#include "../AM/src/MCSPSS.cc"
#include "../AM/src/MCCCRE.cc"
#include "../AM/src/MCRIU.cc"
#include "../AM/src/MCSAD.cc"
#ifndef DISABLE_DFM
#include "../AM/src/DFM.cc"
#endif
#ifndef DISABLE_PCA
#include "../AM/src/PCA.cc"
#endif

//! composite AMs
#include "../AM/src/SumOfAMs.cc"
//! illumination models
#include "../AM/src/GB.cc"
#include "../AM/src/PGB.cc"
#include "../AM/src/RBF.cc"

//! state space models
#include "../SSM/src/Homography.cc"
#include "../SSM/src/Affine.cc"
#include "../SSM/src/Similitude.cc"
#include "../SSM/src/Isometry.cc"
#include "../SSM/src/Transcaling.cc"
#include "../SSM/src/Translation.cc"
#include "../SSM/src/LieHomography.cc"
#include "../SSM/src/CornerHomography.cc"
#include "../SSM/src/SL3.cc"
#include "../SSM/src/Spline.cc"
#include "../SSM/src/ProjectiveBase.cc"
#include "../SSM/src/SSMEstimatorParams.cc"
#include "../SSM/src/SSMEstimator.cc"

//! utilities
#include "../Utilities/src/histUtils.cc"
#include "../Utilities/src/imgUtils.cc"
#include "../Utilities/src/warpUtils.cc"
#include "../Utilities/src/miscUtils.cc"
#include "../Utilities/src/graphUtils.cc"
#include "../Utilities/src/spiUtils.cc"
#ifndef DISABLE_REGNET
#include "../Utilities/src/netUtils.cc"
#endif

//! parameters
#ifndef ENABLE_ONLY_NT
//! search methods
#include "../SM/src/ESMParams.cc"
#include "../SM/src/ICLKParams.cc"
#include "../SM/src/FCLKParams.cc"
#include "../SM/src/FALKParams.cc"
#include "../SM/src/IALKParams.cc"
#include "../SM/src/PFParams.cc"
#ifndef DISABLE_FLANN
#include "../SM/src/NNParams.cc"
#include "../SM/src/FLANNParams.cc"
#endif
//! composite search methods
#include "../SM/src/RKLTParams.cc"
#include "../SM/src/CascadeParams.cc"
#include "../SM/src/ParallelParams.cc"
#include "../SM/src/PyramidalParams.cc"
#ifndef DISABLE_GRID
#include "../SM/src/GridTrackerFlowParams.cc"
#endif
#endif
#endif
