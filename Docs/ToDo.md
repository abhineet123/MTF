* Windows support - adapt GNU make/cmake build system for Visual Studio
* Matlab front end
* HTML/Javascript front end for web based demo
* Augmented reality application
* Add more feature types (SURF, HOG, BRISK, FAST, ...) to GridTrackerFeat
* Multi channel support for LRSCV
* Multi channel support for NGF
* Multi channel support for ILMs
* Channel wise implementations of multi channel support for all AMs especially MI and CCRE
* Efficient parallelization of costly AMs like MI/CCRE/NGF and SMs like PF and RANSAC
    * OpenMP/TBB for a start and an optional CUDA version too
* Complete KL Divergence (KLD) AM as well as its localized version (LKLD)
* Improve Doxygen documentation
* Selective pixel integration (SPI) support for all AMs and SSMs
* Complete Lie Affine and add CornerAffine and Geometric Affine SSMs
* Complete LieIsometry SSM
* Add CornerSimilitude SSM
* Complete Spline and TPS SSM
* Add piecewise projective and piecewise affine SSMs
