#ifndef STATE_SPACE_MODEL_H
#define STATE_SPACE_MODEL_H

#include "mtf/Macros/common.h"
#include <stdexcept>

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

class StateSpaceModel{

private:
	stringstream excp_msg;

protected:
	int state_size;
	int resx, resy;
	int n_pts;

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

	StateSpaceModel(int _resx, int _resy) : 
		state_size(0),
		resx(_resx), resy(_resy),
		n_pts(resx*resy), 
		identity_jacobian(false), first_iter(false),
		spi_mask(nullptr){
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
	
	virtual const PtsT& getPts(){ return curr_pts; }
	virtual const CornersT& getCorners(){ return curr_corners; }
	virtual const VectorXd& getState(){ return curr_state; }

	virtual const GradPtsT& getGradPts(){ return grad_pts; }
	virtual const HessPtsT& getHessPts(){ return hess_pts; }

	// overloaded accessors that copy the current corners to the provided OpenCV structure 
	// rather than returning it in Eigen format
	virtual void getCorners(cv::Point2d *cv_corners){
		for(int i = 0; i < 4; i++){
			cv_corners[i].x = curr_corners(0, i);
			cv_corners[i].y = curr_corners(1, i);
		}
	}
	virtual void getCorners(cv::Mat &cv_corners){
		for(int i = 0; i < 4; i++){
			cv_corners.at<double>(0, i) = curr_corners(0, i);
			cv_corners.at<double>(1, i) = curr_corners(1, i);
		}
	}
	// --------------------------------------------------------------- //
	// ---------------------------modifiers--------------------------- //
	// --------------------------------------------------------------- //

	virtual void setState(const VectorXd &ssm_state){ssm_not_implemeted(setState);}

	// update the internal state so that the object location is set to the specified corners
	virtual void setCorners(const CornersT& eig_corners){
		throw std::domain_error("SSM :: setCorners:: Not implemented Yet");
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
	virtual void initialize(const CornersT& eig_corners){
		setCorners(eig_corners);
		is_initialized.pts = true;
	}
	// overloaded function to let the user initialize SSM with corners in OpenCV format
	virtual void initialize(const cv::Mat& cv_corners){
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
		ssm_not_implemeted(additiveUpdate);
	}
	virtual void compositionalUpdate(const VectorXd& state_update){
		ssm_not_implemeted(compositionalUpdate);
	}
	virtual void updateGradPts(double grad_eps){
		ssm_not_implemeted(updateGradPts);
	}
	virtual void updateHessPts(double hess_eps){
		ssm_not_implemeted(updateHessPts);
	}
	// compute the state corresponding to the compositional inverse of the transformation
	// described by the given state, i.e. W(inv_state, W(state, pts)) = pts
	virtual void invertState(VectorXd& inv_state, const VectorXd& state){
		ssm_not_implemeted(invertState);
	}

	// --------------------------------------------------------------------------- //
	// ---------------------------interfacing functions--------------------------- //
	// --------------------------------------------------------------------------- //

	//! right multiplies the initial or current ssm jacobian with the provided am jacobian; 
	//! though this can be accomplished by the search method itself with simple matrix multiplication, 
	//! the ssm jacobian often satisfies several constraints (e.g. sparsity) that can be exploited to gain 
	//! computational savings by manually computing the product matrix
	virtual void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad) {
		ssm_not_implemeted(cmptInitPixJacobian);
	}
	virtual void cmptCurrPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad){
		ssm_not_implemeted(cmptCurrPixJacobian);
	}

	virtual void cmptInitWarpedPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad) {
		ssm_not_implemeted(cmptInitWarpedPixJacobian);
	}
	virtual void cmptCurrWarpedPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad) {
		ssm_not_implemeted(cmptCurrWarpedPixJacobian);
	}


	virtual void cmptInitJointInvPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad) {
		ssm_not_implemeted(cmptInitJointInvPixJacobian);
	}
	virtual void cmptCurrJointInvPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pixel_grad) {
		ssm_not_implemeted(cmptInitWarpedPixHessian);
	}
	virtual void cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) {
		ssm_not_implemeted(cmptInitPixHessian);
	}
	virtual void cmptCurrPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) {
		ssm_not_implemeted(cmptCurrPixHessian);
	}

	virtual void cmptInitWarpedPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad){
		ssm_not_implemeted(cmptInitWarpedPixHessian);
	}

	virtual void cmptCurrWarpedPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) {
		ssm_not_implemeted(cmptCurrWarpedPixHessian);
	}

	// computes the warped corners generated by applying the warp corresponding to the given state vector to the given corners
	virtual void applyWarpToCorners(CornersT &out_corners, const CornersT &in_corners,
		const VectorXd &ssm_state){
		ssm_not_implemeted(applyWarpToCorners);
	}
	// computes the warped points generated by applying the warp corresponding to the given state vector to the given points
	virtual void applyWarpToPts(PtsT &out_pts, const PtsT &in_pts,
		const VectorXd &ssm_state){
		ssm_not_implemeted(applyWarpToPts);
	}

	// estimates the state vector whose corresponding transformation, when applied to in_corners
	// produces out_corners; if such a transformation is not exactly possible due to constraints
	// on the SSM state, the resultant corners must be optimal in the least squares sense, i.e. their
	// Euclidean distance from out_corners should be as small as possible while obeying these constraints 
	virtual void estimateWarpFromCorners(VectorXd &state_update, const CornersT &in_corners,
		const CornersT &out_corners){
		ssm_not_implemeted(estimateWarpFromCorners);
	}
	// estimates the state vector whose corresponding transformation, when applied to in_pts
	// produces points that are best fit to out_pts according to estimation_method; 
	virtual void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts, 
		int estimation_method, double ransac_reproj_thresh){
		ssm_not_implemeted(estimateWarpFromPts);
	}

	// --------------------------------------------------------------------------- //
	// ----------------------  miscellaneous functionality  ---------------------- //
	// --------------------------------------------------------------------------- //

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
