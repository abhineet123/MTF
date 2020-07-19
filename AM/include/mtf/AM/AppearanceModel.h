#ifndef MTF_APPEARANCE_MODEL_H
#define MTF_APPEARANCE_MODEL_H

#define am_func_not_implemeted(func_name) \
	throw mtf::utils::FunctonNotImplemented(cv::format("%s :: %s :: Not implemented Yet", name.c_str(), #func_name))

#include "ImageBase.h"
#include "AMParams.h"
#include "mtf/Utilities/excpUtils.h"
#include <memory>

_MTF_BEGIN_NAMESPACE

/**
set of indicator variables to keep track of which dependent state variables need updating and executing
the update expression for any variable only when at least one of the other variables it depends on has
been updated; this can help to increase speed by avoiding repeated computations
*/
struct AMStatus : ImgStatus{
	bool similarity, grad, hess;
	bool sampler;

	AMStatus() : ImgStatus(){ clear(); }

	void set(){
		setPixState();
		similarity = grad = hess = true;
		sampler = true;
	}
	void clear(){
		clearPixState();
		similarity = grad = hess = false;
		sampler = false;
	}
};

/**
Distance functor for FLANN
*/
struct AMDist{
	typedef double ElementType;
	typedef double ResultType;
	const string name;
	AMDist(const string &_name) : name(_name){}
	virtual ~AMDist() {}
	/**
	computes the distance / dissimilarity between two patches where each is codified or represented
	by a suitable distance feature computed using updateDistFeat
	*/
	virtual double operator()(const double* a, const double* b,
		size_t dist_size, double worst_dist = -1) const {
		am_func_not_implemeted(distance_functor);
	}
};

/**
Similarity function that indicates how well a candidate warped patch matches the template.
Registration based tracking is expressed as the solution to the following optimization problem:
p_t = argmax(p_ssm, p_am) f(I_0(x), I_t(w(x, p_ssm)), p_am)
where p_ssm and p_am are respectively the geometric and photometric parameters to be optimized.
Appearance Model corresponds to the similarity function f.
*/
class AppearanceModel : public ImageBase{
public:
	/** name of the appearance model */
	string name;

	/** default and value constructor */
	explicit AppearanceModel(const AMParams *params = nullptr, 
		const int _n_channels = 1) :
		ImageBase(params, _n_channels), f(0), state_size(0),
		first_iter(false), spi_mask(nullptr){}
	/** destructor */
	virtual ~AppearanceModel(){}

	/** accessor methods */
	virtual int getStateSize() const{ return 0; }
	virtual double getSimilarity() const{ return f; }
	/**
	returns a normalized version of the similarity that lies between 0 and 1
	and can be interpreted as the likelihood that the current patch represents 
	the same object as the initial patch
	*/
	virtual double getLikelihood() const{ am_func_not_implemeted(getLikelihood); }
	virtual const VectorXd& getState(){ return p_am; }
	virtual const RowVectorXd& getInitGrad() const{ return df_dI0; }
	virtual const RowVectorXd& getCurrGrad() const{ return df_dIt; }

	/** modifier methods */
	virtual void setSimilarity(double _similarity){ f = _similarity; }
	virtual void setInitGrad(const RowVectorXd &df_dI){ df_dI0 = df_dI; }
	virtual void setCurrGrad(const RowVectorXd &df_dI){ df_dIt = df_dI; }


	/**
	methods to initialize the state variables - to be called once when the tracker is initialized.
	if any of these are reimplemented, there should be a statement there copying the computed value in the "init"
	variable to the corresponding "curr" variable so the two have the same value after this function is called;

	Note for the SM: the "initialize" function for any state variable whose "update" function will be called later should be called
	once from the SM's own initialize function even if the initial value of that variable will not be used later;
	this is because updating the state variable may involve some computations which need to be performed only once and the AM is free to delegate
	any such computations to the respective "initialize" function to avoid repeating them in the "update" function which can have a negative impact on performance;
	thus if this function is not called, the results of these computations that are needed by the "update" function will remain uncomputed leading to undefined behavior;

	also the initialize functions have a boolean indicator parameter called "is_initialized" which defaults to true but should be set to false
	if they are called only to update the internal state to take into account any changes to the initial template, i.e. if they are called
	again after the first call to initialize the state (when it should be left to true)
	*/
	virtual void initializeSimilarity(){ am_func_not_implemeted(initializeSimilarity); }
	virtual void initializeGrad() { am_func_not_implemeted(initializeGrad); }
	/**
	// even though the Hessian of the error norm w.r.t. pixel values is not a state variable (since it does not need
	// to be computed separately to get the Hessian w.r.t SSM), this function is provided as a place to perform
	// any one-time computations that may help to decrease the runtime cost of the interfacing function that computes this Hessian
	*/
	virtual void initializeHess(){ am_func_not_implemeted(initializeHess); }

	virtual void reinitialize(){
		if(is_initialized.similarity){ initializeSimilarity(); }
		if(is_initialized.grad){ initializeGrad(); }
		if(is_initialized.hess){ initializeHess(); }
	}

	/**
	functions for updating state variables when a new image arrives
	*/
	/**
	prereq_only should be left to true if update is only called to compute the prerequisites
	for the two gradient functions and the actual value of similarity is not needed
	*/
	virtual void updateSimilarity(bool prereq_only = true){ am_func_not_implemeted(updateSimilarity); }
	virtual void updateState(const VectorXd& state_update){}
	virtual void invertState(VectorXd& inv_p, const VectorXd& p){}
	virtual void updateInitGrad(){ am_func_not_implemeted(updateInitGrad); }
	virtual void updateCurrGrad() { am_func_not_implemeted(updateCurrGrad); }
	virtual void updateParamGrad() { am_func_not_implemeted(updateParamGrad); }

	//virtual void updateInitHess() { am_func_not_implemeted(updateInitHess); }
	//virtual void updateCurrHess() { am_func_not_implemeted(updateCurrHess); }

	/**
	// ----- interfacing functions that take pixel jacobians/Hessians w.r.t. SSM parameters and combine them with AM jacobians/Hessians w.r.t. pixel values ---- //
	*/
	/** to produce corresponding jacobians w.r.t. SSM parameters */
	virtual void cmptInitJacobian(RowVectorXd &df_dp, const MatrixXd &dI0_dpssm){
		assert(dI0_dpssm.rows() == n_pix && dI0_dpssm.cols() == df_dp.cols());
		df_dp.noalias() = df_dI0 * dI0_dpssm;
	}
	virtual void cmptCurrJacobian(RowVectorXd &df_dp, const MatrixXd &dIt_dpssm){
		assert(dIt_dpssm.rows() == n_pix && dIt_dpssm.cols() == df_dp.cols());
		df_dp.noalias() = df_dIt * dIt_dpssm;
	}
	/**
	multiplies the gradients of the current error norm w.r.t. current and initial pixel values with the respective pixel jacobians and computes
	the difference between the resultant jacobians of the current error norm  w.r.t. external (e.g. SSM) parameters
	though this can be done by the search method itself, a dedicated method is provided to take advantage of any special constraints to
	speed up the computations; if mean of the two pixel jacobians is involved in this computation, it is stored in the provided matrix to
	avoid recomputing it while computing the error norm hessian
	*/
	virtual void cmptDifferenceOfJacobians(RowVectorXd &df_dp_diff,
		const MatrixXd &dI0_dpssm, const MatrixXd &dIt_dpssm){
		df_dp_diff.noalias() = (df_dIt * dIt_dpssm) - (df_dI0 * dI0_dpssm);
	}

	/**
	compute the S x S Hessian of the error norm using the supplied N x S Jacobian of pixel values w.r.t. external parameters;
	compute the approximate Hessian by ignoring the second order terms
	*/
	virtual void cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dpssm){
		am_func_not_implemeted(cmptInitHessian(first order));
	}
	virtual void cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dpssm){
		am_func_not_implemeted(cmptCurrHessian(first order));
	}
	virtual void cmptSelfHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dpssm) {
		am_func_not_implemeted(cmptSelfHessian(first order));
	}

	/** compute the exact Hessian by considering the second order terms too */
	virtual void cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dpssm, const MatrixXd &d2I0_dpssm2){
		am_func_not_implemeted(cmptInitHessian(second order));
	}
	virtual void cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dpssm,
		const MatrixXd &d2It_dpssm2){
		am_func_not_implemeted(cmptCurrHessian(second order));
	}
	virtual void cmptSelfHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dpssm,
		const MatrixXd &d2I_dpssm2) {
		am_func_not_implemeted(cmptSelfHessian(second order));
	}

	/** analogous to cmptDifferenceOfJacobians except for computing the mean of the current and initial Hessians */
	virtual void cmptSumOfHessians(MatrixXd &d2f_dp2_sum,
		const MatrixXd &dI0_dpssm, const MatrixXd &dIt_dpssm){
		int pssm_size = static_cast<int>(dIt_dpssm.cols());

		assert(d2f_dp2_sum.rows() == pssm_size && d2f_dp2_sum.cols() == pssm_size);
		MatrixXd d2f_dp2_0(pssm_size, pssm_size);
		cmptInitHessian(d2f_dp2_0, dI0_dpssm);

		MatrixXd d2f_dp2_t(pssm_size, pssm_size);
		cmptCurrHessian(d2f_dp2_t, dIt_dpssm);

		d2f_dp2_sum = d2f_dp2_0 + d2f_dp2_t;
	}
	virtual void cmptSumOfHessians(MatrixXd &d2f_dp2_sum,
		const MatrixXd &dI0_dpssm, const MatrixXd &dIt_dpssm,
		const MatrixXd &d2I0_dpssm2, const MatrixXd &d2It_dpssm2){
		int pssm_size = static_cast<int>(dIt_dpssm.cols());
		assert(d2f_dp2_sum.rows() == pssm_size && d2f_dp2_sum.cols() == pssm_size);
		MatrixXd d2f_dp2_0(pssm_size, pssm_size);
		cmptInitHessian(d2f_dp2_0, dI0_dpssm, d2I0_dpssm2);

		MatrixXd d2f_dp2_t(pssm_size, pssm_size);
		cmptCurrHessian(d2f_dp2_t, dIt_dpssm, d2It_dpssm2);

		d2f_dp2_sum = d2f_dp2_0 + d2f_dp2_t;
	}

	virtual void estimateOpticalFlow(std::vector<cv::Point2f> &curr_pts,
		const cv::Mat &prev_img, const std::vector<cv::Point2f> &prev_pts,
		const cv::Size &win_size, unsigned int n_pts, int max_iters,
		double term_eps, bool const_grad = true) const{
		am_func_not_implemeted(estimateOpticalFlow);
	}

	virtual void setSPIMask(const bool *_spi_mask){ spi_mask = _spi_mask; }
	virtual const bool* getSPIMask() const{ return spi_mask; }
	virtual void clearSPIMask(){ spi_mask = nullptr; }

	/**
	should be overridden by an implementing class once it implements SPI functionality
	for all functions where it makes logical sense
	*/
	virtual bool supportsSPI() const { return false; }

	/**
	should be called before performing the first iteration on a new image to indicate that the image
	has changed since the last time the update funcvtions were called
	*/
	virtual void setFirstIter(){ first_iter = true; }
	/** should be called after the first iteration on a new frame is done */
	virtual void clearFirstIter(){ first_iter = false; }

	ImgStatus* isInitialized() override { return &is_initialized; }
	virtual void setInitStatus(){ is_initialized.set(); }
	virtual void clearInitStatus(){ is_initialized.clear(); }

	/**
	return false if the similarity function f is not symmetrical, i.e. f(a,b) != f(b, a);
	also applies to the distance functor so the two should be consistent;
	*/
	virtual bool isSymmetrical() const{ return true; }

	/**
	optional function to incorporate online learning or adaptation of the model used to represent
	the appearance  of the object being tracked
	this should be called with the final location of the object (obtained by the search process) in each frame
	*/
	virtual void updateModel(const PtsT& curr_pts){ am_func_not_implemeted(updateModel); }

	// ------------------------ Distance Feature ------------------------ //

	/** to be called once during initialization if any of the distance feature functionality is to be used */
	virtual void initializeDistFeat() {
		am_func_not_implemeted(initializeDistFeat);
	}
	/**
	 computes a "distance" vector using the current image patch such that,
	 when the distance vectors corresponding to two patches are passed to the distance operator above,
	 it uses these to compute a scalar that measures the distance or dissimilarity between the two patches;
	 this distance vector should be designed so as to offload as much computation as possible from the
	 distance operator, i.e. every computation that depends only on the current patch should be performed here
	 and the results should be stored, suitably coded, in the distannce vector
	 where they will be decoded and used to compute the distance measure
	 */
	virtual void updateDistFeat() {
		am_func_not_implemeted(updateDistFeat);
	}
	virtual const double* getDistFeat(){
		am_func_not_implemeted(getDistFeat);
	}
	/**
	overloaded version to write the distance feature directly to a row (or column) of a matrix
	storing the distance features corresponding to several patches;
	*/
	virtual void updateDistFeat(double* feat_addr) {
		am_func_not_implemeted(updateDistFeat);
	}
	/** returns the size of the distance vector */
	virtual unsigned int getDistFeatSize() {
		am_func_not_implemeted(getDistFeatSize);
	}
	virtual const AMDist* getDistFunc() {
		am_func_not_implemeted(getDistFunc);
	}

	// -------------------------------------------------------------------------- //
	// --------------------------- Stochastic Sampler --------------------------- //
	// -------------------------------------------------------------------------- //

	virtual void initializeSampler(const VectorXd &state_sigma,
		const VectorXd &state_mean){
		am_func_not_implemeted(initializeSampler(VectorXd, VectorXd));
	}
	virtual void setSampler(const VectorXd &state_sigma,
		const VectorXd &state_mean){
		am_func_not_implemeted(setSampler);
	}
	virtual void setSamplerMean(const VectorXd &mean){
		am_func_not_implemeted(setSamplerMean(VectorXd));
	}
	virtual void setSamplerSigma(const VectorXd &std){
		am_func_not_implemeted(setSamplerSigma(VectorXd));
	}
	virtual void getSamplerMean(VectorXd &mean){
		am_func_not_implemeted(getSamplerMean);
	}
	virtual void getSamplerSigma(VectorXd &std){
		am_func_not_implemeted(getSamplerMean);
	}
	virtual void generatePerturbation(VectorXd &perturbation){
		am_func_not_implemeted(generatePerturbation);
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
	/**
	f(I_0, I_t, p_am): R(N) x R(N) x R(K) -> R
	measures the similarity between the current (I_t) and
	initial (I_0) patches using the photometric parameters (p_am)
	this is the quantity to be maximized by the optimization process
	*/
	double f;

	/**
	1 x N gradients of the similarity
	w.r.t. initial and current pixel values;
	*/
	RowVectorXd df_dI0, df_dIt;

	/** parameters of the photomtric model */
	VectorXd p_am;

	/** size of p_am, i.e. number of parameters in the similarity function */
	int state_size;

	/** 1 x K Jacobian of the similarity function w.r.t. its parameters */
	RowVectorXd df_dpam;

	/** K x K Hessian of the similarity w.r.t. its parameters */
	MatrixXd d2f_dpam2;

	/**
	1 x N gradient of the similarity w.r.t. illumination model
	if such a one is used: f->f(I_0, g(I_t, p_am))
	*/
	RowVectorXd df_dg0, df_dgt;

	/**
	K x N cross Hessian of the similarity
	w.r.t. photomtric parameters p_am and the current patch  I_t
	assuming again that an illumination model is in use;
	*/
	MatrixXd d2f_dpam_dIt;

	/**
	these NxN Hessians of the similarity wrt pixel values are usually not stored or computed explicitly because:
	1. the matrices are just too large for higher sampling resolutions
	2. they are often very sparse so allocating so much space is wasteful
	3. computing the Hessian wrt SSM parameters by multiplying this matrix with the SSM Hessian is highly inefficient is highly inefficient
	*/
	MatrixXd d2f_dI02, d2f_dIt2;

	/**
	indicator variable that can be set by iterative search methods to indicate if the initial or first iteration is being run on the current image;
	can be used to perform some costly operations/updates only once per frame rather than at every iteration
	*/
	bool first_iter;

	/**
	pixels corresponding to false entries in the mask will be ignored in all respective computations where pixel values are used;
	it is up to the AM to do this in a way that makes sense; since none of the state variables are actually being resized they will
	still have entries corresponding to these ignored pixels but the AM s at liberty to put anything there assuming
	that the SM will not use these entries in its own computations; this is why all of these have default implementations that simply ignore the mask;
	these can be used by the AM when the non masked entries of the computed variable do not depend on the masked pixels;
	*/
	const bool *spi_mask;

	/**
	indicator variables used to keep track of which state variables have been initialized;
	*/
	AMStatus is_initialized;
};

_MTF_END_NAMESPACE

#endif



