#ifndef MTF_IMAGE_BASE_H
#define MTF_IMAGE_BASE_H

#define mc_not_implemeted(func_name, n_channels) \
	throw mtf::utils::FunctonNotImplemented(cv::format("%s :: %d channel images are not supported yet", #func_name,n_channels))

#define WARPED_GRAD_EPS 1e-8
#define GRAD_EPS 1e-8
#define HESS_EPS 1
#define UCHAR_INPUT false


#define PIX_MAX 255.0
#define PIX_MIN 0.0

#include "mtf/Macros/common.h"
#include "mtf/Utilities/excpUtils.h"

_MTF_BEGIN_NAMESPACE

struct ImgParams{
	//! horizontal and vertical sampling resolutions
	int resx, resy;
	//! numerical increment/decrement used for computing image hessian and gradient
	//! using the method of finite differences
	double grad_eps, hess_eps;
	bool uchar_input;
	ImgParams(int _resx, int _resy,
		double _grad_eps = GRAD_EPS,
		double _hess_eps = HESS_EPS,
		bool _uchar_input = UCHAR_INPUT);
	ImgParams(const ImgParams *img_params = nullptr);
};

// set of indicator variables to keep track of which dependent state variables need updating and executing
// the update expression for any variable only when at least one of the other variables it depends on has 
// been updated; this can help to increase speed by avoiding repeated computations
struct ImgStatus{
	bool pix_vals, pix_grad, pix_hess;

	ImgStatus(){ clearPixState(); }

	void setPixState(){
		pix_vals = pix_grad = pix_hess = true;
	}
	void clearPixState(){
		pix_vals = pix_grad = pix_hess = false;
	}
};
class ImageBase{

public:
	/** convenience type if floating point grayscale image is used as input by the AM */
	typedef EigImgT ImageT;

	explicit ImageBase(const ImgParams *img_params = nullptr,
		const int _n_channels = 1);
	virtual ~ImageBase(){}

	//! return the type of OpenCV Mat image the AM requires as input; 
	//! typically either CV_32FC3 or CV_32FC1
	virtual int inputType() const {
		return uchar_input ?
			n_channels == 1 ? CV_8UC1 : CV_8UC3 :
			n_channels == 1 ? CV_32FC1 : CV_32FC3;
	}

	//! accessor methods; these are not defined as 'const' since an appearance model may like to
	//! make some last moment changes to the variable being accessed before returning it to can avoid any 
	//! unnecessary computations concerning the variable (e.g. in its 'update' function) unless it is actually accessed;
	virtual const cv::Mat& getCurrImg() const{ return curr_img_cv; }
	virtual unsigned int getImgHeight() const{ return img_height; }
	virtual unsigned int getImgWidth() const{ return img_width; }
	virtual unsigned int getResX() const{ return resx; }
	virtual unsigned int getResY() const{ return resy; }
	virtual unsigned int getNPix() const{ return n_pix; }
	virtual unsigned int getNChannels() const{ return n_channels; }
	virtual unsigned int getPatchSize() const{ return patch_size; }
	virtual double getGradOffset() const{ return grad_eps; }
	virtual double getHessOffset() const{ return hess_eps; }
	virtual const PixValT& getInitPixVals() const{ return I0; }
	virtual const PixGradT& getInitPixGrad() const{ return dI0_dx; }
	virtual const PixHessT& getInitPixHess() const{ return d2I0_dx2; }

	virtual const PixValT& getCurrPixVals() const{ return It; }
	virtual const PixGradT& getCurrPixGrad() const{ return dIt_dx; }
	virtual const PixHessT& getCurrPixHess() const{ return d2It_dx2; }

	//! modifier methods;
	virtual void setCurrImg(const cv::Mat &cv_img);
	virtual void setInitPixVals(const PixValT &pix_vals){ I0 = pix_vals; }
	virtual void setInitPixGrad(const PixGradT &pix_grad){ dI0_dx = pix_grad; }
	virtual void setInitPixHess(const PixHessT &pix_hess){ d2I0_dx2 = pix_hess; }

	virtual void setCurrPixVals(const PixValT &pix_vals){ It = pix_vals; }
	virtual void setCurrPixGrad(const PixGradT &pix_grad){ dIt_dx = pix_grad; }
	virtual void setCurrPixHess(const PixHessT &pix_hess){ d2It_dx2 = pix_hess; }

	//! initialization methods - to be called once when the tracker is initialized
	virtual void initializePixVals(const PtsT& init_pts);
	//! functions to compute image differentials (gradient and hessian) are overloaded since there are two ways to define them:
	//! 1. differential of the warped image: the image is warped first, then its differential is computed at the base (unwarped) locations
	//! 2. warp of the image differential:  differential is computed in the image coordinates and evaluated at the given (presumably warped) locations
	//! 1. gradient of the warped image
	virtual void initializePixGrad(const GradPtsT &warped_offset_pts);
	//! 2. warp of the image gradient
	virtual void initializePixGrad(const PtsT &init_pts);

	//! 1. hessian of the warped image
	virtual void initializePixHess(const PtsT& init_pts, const HessPtsT &warped_offset_pts);
	//! 2. warp of the image hessian
	virtual void initializePixHess(const PtsT &init_pts);

	// -------- functions for updating state variables when a new image arrives -------- //
	virtual void updatePixVals(const PtsT& curr_pts);

	virtual void updatePixGrad(const GradPtsT &warped_offset_pts);
	virtual void updatePixGrad(const PtsT &curr_pts);

	virtual void updatePixHess(const PtsT &curr_pts);
	virtual void updatePixHess(const PtsT& curr_pts, const HessPtsT &warped_offset_pts);

	//! general utility function to extract raw pixel values from the current image at the specified points; 
	//! might be useful for visualization purposes as the curr_pix_vals might not have raw pixel values;
	virtual void extractPatch(VectorXd &pix_vals, const PtsT& curr_pts);
	//! returning variant
	virtual VectorXd getPatch(const PtsT& curr_pts);

	virtual ImgStatus* isInitialized() = 0;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	//! Eigen structure shaing memory with the OpenCV image used by default with grayscale inputs
	EigImgT curr_img;
	//! OpenCV image used by default with multi channel inputs
	cv::Mat curr_img_cv;

	//! height and width of the input images
	unsigned int img_height, img_width;
	//! horizontal and vertical sampling resolutions for the object patch
	unsigned int resx, resy;
	//! no. of pixels in the sampled image patch to be tracked	
	unsigned int n_pix;
	//! no. of values that represent each pixel in the sampled patch
	//! defined as a constant to enable compile time optimizations
	const unsigned int n_channels;
	//! size of the vector that represents the object patch in the image, typically n_pix*n_channels
	unsigned int patch_size;

	//! let N = n_pix = no. of pixels and C = n_channels = no. of channels

	//! pixel values for all channels in the object patch being tracked (flattened as a vector)	
	PixValT I0, It;
	//! (N*C) x 2 jacobian of pixel values in the warped image w.r.t. pixel coordinate locations
	//! aka gradient of the warped image or warp of the gradient image depending on the search method
	PixGradT dI0_dx, dIt_dx;
	//! 4 x (N*C) Hessian of pixel values in the warped image w.r.t. pixel coordinate locations;
	//! each column of this matrix contains the 2x2 hessian for the respective location (for each channel) flasttened in the column major order;
	PixHessT d2I0_dx2, d2It_dx2;
	//! additive and multiplicative factors for normalizing pixel values
	double pix_norm_add, pix_norm_mult;
	//! incremented once during initialization and thereafter everytime the template is updated
	unsigned int frame_count;
	//! offsets to use for computing the numerical image gradient and hessian
	double grad_eps, hess_eps;
	//! use 8 bit unsigned integral images as input
	bool uchar_input;
};

_MTF_END_NAMESPACE

#endif



