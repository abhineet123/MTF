#ifndef MTF_STATE_SPACE_MODEL_H
#define MTF_STATE_SPACE_MODEL_H

#include "mtf/Macros/common.h"
#include "mtf/Utilities/excpUtils.h"
#include "SSMEstimatorParams.h"

#define ssm_func_not_implemeted(func_name) \
	throw mtf::utils::FunctonNotImplemented(cv::format("%s :: %s :: Not implemented Yet", name.c_str(), #func_name))

#define validate_ssm_state(state_vec)\
	assert(state_vec.size() == state_size)

#define validate_ssm_jacobian(jacobian_prod, pix_jacobian)\
	assert(jacobian_prod.rows() == n_pts*n_channels && jacobian_prod.cols() == state_size);\
	assert(pix_jacobian.rows() == n_pts*n_channels)

#define validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad)\
	assert(pix_hess_ssm.rows() == state_size*state_size && pix_hess_ssm.cols() == n_pts*n_channels);\
	assert(pix_grad.rows() == n_pts*n_channels && pix_hess_coord.cols() == n_pts*n_channels)

_MTF_BEGIN_NAMESPACE

struct SSMStatus{
	bool pts, grad_pts, hess_pts, sampler;
	SSMStatus(){ clear(); }
	void set(){
		pts = grad_pts = hess_pts = sampler = true;
	}
	void clear(){
		pts = grad_pts = hess_pts = sampler = false;
	}
};

struct SSMParams{
	//! horizontal and vertical sampling resolutions
	int resx, resy;
	SSMParams(int _resx, int _resy) :
		resx(_resx), resy(_resy){}
	SSMParams(const SSMParams *ssm_params = nullptr) :
		resx(MTF_RES), resy(MTF_RES){
		if(ssm_params){
			resx = ssm_params->resx;
			resy = ssm_params->resy;
		}
	}
};

class StateSpaceModel{

protected:
	int state_size;
	int resx, resy;
	int n_pts;
	int n_channels;

	// parameters defining the SSM state
	VectorXd curr_state;

	// grid points and corners that define the object's location in
	// the frame
	PtsT init_pts, curr_pts;
	CornersT init_corners, curr_corners;
	GradPtsT grad_pts;
	HessPtsT hess_pts;
	bool identity_jacobian;

public:
	string name;

	StateSpaceModel(const SSMParams *params) :
		state_size(0), resx(MTF_RES), resy(MTF_RES),
		identity_jacobian(false), first_iter(false),
		spi_mask(nullptr){
		if(params) {
			if(params->resx <= 0 || params->resy <= 0) {
				throw std::invalid_argument("StateSpaceModel::Invalid sampling resolution provided");
			}
			resx = params->resx;
			resy = params->resy;
		}
		n_pts = resx*resy;
		init_pts.resize(Eigen::NoChange, n_pts);
		curr_pts.resize(Eigen::NoChange, n_pts);
	}
	virtual ~StateSpaceModel(){}

	// --------------------------------------------------------------- //
	// ---------------------------accessors--------------------------- //
	// --------------------------------------------------------------- //

	virtual int getStateSize(){ return state_size; }
	virtual int getResX(){ return resx; }
	virtual int getResY(){ return resy; }
	virtual int getNPts(){ return n_pts; }
	virtual int getNChannels(){ return n_channels; }

	virtual const PtsT& getPts(){ return curr_pts; }
	virtual const CornersT& getCorners(){ return curr_corners; }
	virtual const VectorXd& getState(){ return curr_state; }

	virtual const GradPtsT& getGradPts(){ return grad_pts; }
	virtual const HessPtsT& getHessPts(){ return hess_pts; }

	// overloaded accessors that copy the current corners to the provided OpenCV structure 
	// rather than returning it in Eigen format
	virtual void getCorners(cv::Point2d *cv_corners){
		for(int corner_id = 0; corner_id < 4; ++corner_id){
			cv_corners[corner_id].x = curr_corners(0, corner_id);
			cv_corners[corner_id].y = curr_corners(1, corner_id);
		}
	}
	virtual void getCorners(cv::Mat &cv_corners){
		for(int corner_id = 0; corner_id < 4; ++corner_id){
			cv_corners.at<double>(0, corner_id) = curr_corners(0, corner_id);
			cv_corners.at<double>(1, corner_id) = curr_corners(1, corner_id);
		}
	}
	// --------------------------------------------------------------- //
	// ---------------------------modifiers--------------------------- //
	// --------------------------------------------------------------- //

	virtual void setNChannels(int _n_channels){ n_channels = _n_channels; }

	virtual void setState(const VectorXd &ssm_state){
		ssm_func_not_implemeted(setState);
	}

	// update the internal state so that the object location is set to the specified corners
	virtual void setCorners(const CornersT& eig_corners){
		ssm_func_not_implemeted(setCorners);
	}
	// overloaded variant to accept corners in OpenCV Mat format
	virtual void setCorners(const cv::Mat& cv_corners){
		CornersT eig_corners;
		for(int i = 0; i < 4; i++){
			eig_corners(0, i) = cv_corners.at<double>(0, i);
			eig_corners(1, i) = cv_corners.at<double>(1, i);
		}
		setCorners(eig_corners);
	}
	// initialize the internal state variables; an alias for setCorners;
	virtual void initialize(const CornersT& eig_corners, int _n_channels = 1){
		setNChannels(_n_channels);
		setCorners(eig_corners);
		is_initialized.pts = true;
	}
	// overloaded function to let the user initialize SSM with corners in OpenCV format
	virtual void initialize(const cv::Mat& cv_corners, int _n_channels = 1){
		setNChannels(_n_channels);
		setCorners(cv_corners);
		is_initialized.pts = true;
	}

	virtual void initializeGradPts(double grad_eps){
		if(!is_initialized.grad_pts){
			grad_pts.resize(Eigen::NoChange, n_pts);
		}
		updateGradPts(grad_eps);
		is_initialized.grad_pts = true;
	}
	virtual void initializeHessPts(double hess_eps){
		if(!is_initialized.hess_pts){
			hess_pts.resize(Eigen::NoChange, n_pts);
		}
		updateHessPts(hess_eps);
		is_initialized.hess_pts = true;
	}
	// functions to update internal state variables
	virtual void additiveUpdate(const VectorXd& state_update){
		ssm_func_not_implemeted(additiveUpdate);
	}
	virtual void compositionalUpdate(const VectorXd& state_update){
		ssm_func_not_implemeted(compositionalUpdate);
	}
	virtual void updateGradPts(double grad_eps){
		ssm_func_not_implemeted(updateGradPts);
	}
	virtual void updateHessPts(double hess_eps){
		ssm_func_not_implemeted(updateHessPts);
	}
	// compute the state corresponding to the compositional inverse of the transformation
	// described by the given state, i.e. W(inv_state, W(state, pts)) = pts
	virtual void invertState(VectorXd& inv_state, const VectorXd& state){
		ssm_func_not_implemeted(invertState);
	}

	// --------------------------------------------------------------------------- //
	// ---------------------------interfacing functions--------------------------- //
	// --------------------------------------------------------------------------- //

	//! right multiplies the initial or current ssm jacobian with the provided am jacobian; 
	//! though this can be accomplished by the search method itself with simple matrix multiplication, 
	//! the ssm jacobian often satisfies several constraints (e.g. sparsity) that can be exploited to gain 
	//! computational savings by manually computing the product matrix
	virtual void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad) {
		ssm_func_not_implemeted(cmptInitPixJacobian);
	}
	virtual void cmptPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad){
		ssm_func_not_implemeted(cmptPixJacobian);
	}
	virtual void cmptWarpedPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad) {
		ssm_func_not_implemeted(cmptWarpedPixJacobian);
	}
	virtual void cmptApproxPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad) {
		ssm_func_not_implemeted(cmptApproxPixJacobian);
	}
	virtual void cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) {
		ssm_func_not_implemeted(cmptInitPixHessian);
	}
	virtual void cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) {
		ssm_func_not_implemeted(cmptPixHessian);
	}
	virtual void cmptWarpedPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad){
		ssm_func_not_implemeted(cmptWarpedPixHessian);
	}
	virtual void cmptApproxPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) {
		ssm_func_not_implemeted(cmptApproxPixHessian);
	}

	// computes the warped corners generated by applying the warp corresponding to the given state vector to the given corners
	virtual void applyWarpToCorners(CornersT &out_corners, const CornersT &in_corners,
		const VectorXd &ssm_state){
		ssm_func_not_implemeted(applyWarpToCorners);
	}
	// computes the warped points generated by applying the warp corresponding to the given state vector to the given points
	virtual void applyWarpToPts(PtsT &out_pts, const PtsT &in_pts,
		const VectorXd &ssm_state){
		ssm_func_not_implemeted(applyWarpToPts);
	}
	// return SSM state vector p corresponding to the identity warp, i.e. S(x, p)=x;
	virtual void getIdentityWarp(VectorXd &p){
		ssm_func_not_implemeted(applyWarpToPts);
	}
	// compute SSM state vector p12 that corresponds to the composition of p1 and p2, 
	// i.e. S(x, p12) = S(S(x, p1), p2);
	virtual void composeWarps(VectorXd &p12, const VectorXd &p1,
		const VectorXd &p2){
		ssm_func_not_implemeted(applyWarpToPts);
	}


	// estimates the state vector whose corresponding transformation, when applied to in_corners
	// produces out_corners; if such a transformation is not exactly possible due to constraints
	// on the SSM state, the resultant corners must be optimal in the least squares sense, i.e. their
	// Euclidean distance from out_corners should be as small as possible while obeying these constraints 
	virtual void estimateWarpFromCorners(VectorXd &state_update, const CornersT &in_corners,
		const CornersT &out_corners){
		ssm_func_not_implemeted(estimateWarpFromCorners);
	}
	// overloaded variant that takes OpenCV Mat instead
	virtual void estimateWarpFromCorners(VectorXd &state_update, const cv::Mat &in_corners_cv,
		const cv::Mat &out_corners_cv){
		CornersT in_corners, out_corners;
		for(int i = 0; i < 4; ++i){
			in_corners(0, i) = in_corners_cv.at<double>(0, i);
			in_corners(1, i) = in_corners_cv.at<double>(1, i);
			out_corners(0, i) = out_corners_cv.at<double>(0, i);
			out_corners(1, i) = out_corners_cv.at<double>(1, i);
		}
		estimateWarpFromCorners(state_update, in_corners, out_corners);
	}
	typedef SSMEstimatorParams EstimatorParams;

	// estimates the state vector whose corresponding transformation, when applied to in_pts
	// produces points that are best fit to out_pts according to estimation_method; 
	virtual void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		const EstimatorParams &est_params){
		ssm_func_not_implemeted(estimateWarpFromPts);
	}

	// -------------------------------------------------------------------------- //
	// --------------------------- Stochastic Sampler --------------------------- //
	// -------------------------------------------------------------------------- //
		
	virtual void initializeSampler(const VectorXd &state_sigma, 
		const VectorXd &state_mean){
		ssm_func_not_implemeted(initializeSampler(VectorXd, VectorXd));
	}
	virtual void setSampler(const VectorXd &state_sigma,
		const VectorXd &state_mean){
		ssm_func_not_implemeted(setSampler);
	}
	virtual void setSamplerMean(const VectorXd &mean){
		ssm_func_not_implemeted(setSamplerMean(VectorXd));
	}
	virtual void setSamplerSigma(const VectorXd &std){
		ssm_func_not_implemeted(setSamplerSigma(VectorXd));
	}
	virtual VectorXd getSamplerSigma(){ ssm_func_not_implemeted(getSamplerSigma); }
	virtual VectorXd getSamplerMean(){ ssm_func_not_implemeted(getSamplerMean); }


	// use Random Walk model to generate perturbed sample
	virtual void compositionalRandomWalk(VectorXd &perturbed_state, const VectorXd &base_state){
		ssm_func_not_implemeted(compositionalRandomWalk);
	}
	virtual void additiveRandomWalk(VectorXd &perturbed_state, const VectorXd &base_state){
		ssm_func_not_implemeted(additiveRandomWalk);
	}
	// use first order Auto Regressive model to generate perturbed sample
	virtual void compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
		const VectorXd &base_state, const VectorXd &base_ar, double a = 0.5){
		ssm_func_not_implemeted(compositionalAutoRegression1);
	}
	// use first order Auto Regressive model to generate perturbed sample
	virtual void additiveAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
		const VectorXd &base_state, const VectorXd &base_ar, double a = 0.5){
		ssm_func_not_implemeted(additiveAutoRegression1);
	}
	virtual void generatePerturbation(VectorXd &perturbation){
		ssm_func_not_implemeted(generatePerturbation);
	}
	virtual void generatePerturbedPts(VectorXd &perturbed_pts){
		ssm_func_not_implemeted(generatePerturbedPts);
	}
	virtual void getPerturbedPts(VectorXd &perturbed_pts,
		const VectorXd &state_update){
		ssm_func_not_implemeted(getPerturbedPts);
	}

	virtual void estimateMeanOfSamples(VectorXd &sample_mean,
		const std::vector<VectorXd> &samples, int n_samples){
		ssm_func_not_implemeted(estimateMeanOfSamples);
	}
	virtual void estimateStateSigma(VectorXd &state_sigma, double pix_sigma){
		ssm_func_not_implemeted(estimateStateSigma);
	}

	// --------------------------------------------------------------------------- //
	// ----------------------  miscellaneous functionality  ---------------------- //
	// --------------------------------------------------------------------------- //

	// gradient of the SSM w.r.t. its parameters computed at the given point
	// can be used for estimating the change in SSM parameters needed to produce 
	// a required change in point coordinates in image space
	virtual void getInitPixGrad(PtsT &jacobian_prod, int pix_id){
		ssm_func_not_implemeted(getInitPixGrad);
	}
	virtual void getCurrPixGrad(PtsT &jacobian_prod, int pix_id){
		ssm_func_not_implemeted(getCurrPixGrad);
	}

	SSMStatus is_initialized;
	virtual void setInitStatus(){ is_initialized.set(); }
	virtual void clearInitStatus(){ is_initialized.clear(); }

	// indicator variable that can be set by the search method to indicate when a new frame has been acquired;
	// can be used to perform some costly operations/updates only once per frame rather than at every iteration
	// of the same frame
	bool first_iter;
	//should be called before performing the first iteration on a new frame to indicate that the image
	// that will subsequently be passed to the update functions is a new one, i.e. different from 
	// the one passed the last time these were called
	virtual void setFirstIter(){ first_iter = true; }
	//should be called after the first iteration on a new frame is done
	virtual void clearFirstIter(){ first_iter = false; }

	const bool *spi_mask;
	virtual void setSPIMask(const bool *_spi_mask){ spi_mask = _spi_mask; }
	virtual void clearSPIMask(){ spi_mask = nullptr; }
	virtual bool supportsSPI(){ return false; }// should be overridden by an implementing class once 
	// it implements SPI functionality for all functions where it makes logical sense

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
_MTF_END_NAMESPACE

#endif
