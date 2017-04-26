#ifndef MTF_SSD_BASE_H
#define MTF_SSD_BASE_H

#include "AppearanceModel.h"

#include <boost/random/linear_congruential.hpp>
#include <boost/random/normal_distribution.hpp>

_MTF_BEGIN_NAMESPACE

/**
base class for appearance models that use the negative sum of squared differences ("SSD") or
L2 norm of the difference between the initial and current pixel values (original or modified)
as the similarity measure
*/
struct SSDDist : AMDist{
	typedef bool is_kdtree_distance;

	SSDDist(const string &_name) : AMDist(_name){}
	/**
	* Squared Euclidean distance functor, optimized version
	*/
	/**
	*  Compute the squared Euclidean distance between two vectors.
	*
	*	This is highly optimized, with loop unrolling, as it is one
	*	of the most expensive inner loops.
	*
	*	The computation of squared root at the end is omitted for
	*	efficiency.
	*/
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;
	/**
	*	Partial euclidean distance, using just one dimension. This is used by the
	*	kd-tree when computing partial distances while traversing the tree.
	*
	*	Squared root is omitted for efficiency.
	*/
	double accum_dist(const double& a, const double& b, int) const{
		return (a - b)*(a - b);
	}
private:
	~SSDDist(){}
};
class SSDBase : public AppearanceModel{

public:
	typedef IlluminationModel::PixHessType ILMPixHessT;

	SSDBase(const AMParams *am_params = nullptr,
		const int _n_channels = 1);
	int getStateSize() const override{ return state_size; }
	void initializeSimilarity() override;
	void initializeGrad() override;
	void initializeHess() override{
		if(ilm){
			d2f_dpam2.resize(state_size, state_size);
			d2f_dpam_dIt.resize(state_size, patch_size);
		}
	}
	/**
	nothing is done here since init_grad is same as and shares memory
	with curr_pix_diff that is updated in updateSimilarity
	(which in turn is a prerequisite of this)
	*/
	void updateInitGrad() override{
		if(ilm){
			df_dg0 = df_dI0;
			ilm->cmptPixJacobian(df_dI0.data(), df_dg0.data(), I0.data(), p_am.data());
		}
	}
	double getLikelihood() const override;
	//-----------------------------------------------------------------------------------//
	//-------------------------------update functions------------------------------------//
	//-----------------------------------------------------------------------------------//
	void updateSimilarity(bool prereq_only = true) override;
	void updateState(const VectorXd& state_update) override;
	void invertState(VectorXd& inv_p, const VectorXd& p) override;
	void updateCurrGrad() override;
	void cmptInitJacobian(RowVectorXd &df_dp,
		const MatrixXd &dI0_dpssm) override;
	void cmptCurrJacobian(RowVectorXd &df_dp,
		const MatrixXd &dIt_dpssm) override;
	void cmptDifferenceOfJacobians(RowVectorXd &diff_of_jacobians,
		const MatrixXd &dI0_dpssm, const MatrixXd &dIt_dpssm) override;
	void cmptInitHessian(MatrixXd &init_hessian,
		const MatrixXd &init_pix_jacobian) override;
	void cmptCurrHessian(MatrixXd &curr_hessian,
		const MatrixXd &curr_pix_jacobian) override;
	void cmptSelfHessian(MatrixXd &self_hessian,
		const MatrixXd &curr_pix_jacobian) override{
		cmptCurrHessian(self_hessian, curr_pix_jacobian);
	}
	void cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override{
		cmptSelfHessian(self_hessian, curr_pix_jacobian);
	}
	void cmptSumOfHessians(MatrixXd &sum_of_hessians,
		const MatrixXd &init_pix_jacobian,
		const MatrixXd &curr_pix_jacobian) override;
	void cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
		const MatrixXd &init_pix_hessian) override;
	void cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override;
	void cmptSumOfHessians(MatrixXd &sum_of_hessians,
		const MatrixXd &init_pix_jacobian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &init_pix_hessian, const MatrixXd &curr_pix_hessian) override;

	/**
	Support for FLANN library
	*/
	typedef SSDDist DistType;
	const DistType* getDistPtr() override{
		return new DistType(name);
	}
	void updateDistFeat(double* feat_addr) override{
		int feat_size = getDistFeatSize();
		for(size_t pix = 0; pix < feat_size; pix++) {
			*feat_addr++ = It(pix);
		}
	}
	void initializeDistFeat() override{}
	void updateDistFeat() override{}
	const double* getDistFeat() override{ return getCurrPixVals().data(); }
	unsigned int getDistFeatSize() override{ return n_pix*n_channels; }

	// -------------------------------------------------------------------------- //
	// --------------------------- Stochastic Sampler --------------------------- //
	// -------------------------------------------------------------------------- //

	typedef boost::minstd_rand SampleGenT;
	typedef boost::normal_distribution<double> SampleDistT;
	typedef SampleDistT::param_type DistParamT;

	std::vector<SampleGenT> rand_gen;
	std::vector<SampleDistT> rand_dist;
	VectorXd state_perturbation;

	void initializeSampler(const VectorXd &state_sigma,
		const VectorXd &state_mean) override;
	void setSampler(const VectorXd &state_sigma,
		const VectorXd &state_mean) override;
	void setSamplerMean(const VectorXd &mean) override;
	void setSamplerSigma(const VectorXd &std) override;
	void getSamplerMean(VectorXd &mean) override;
	void getSamplerSigma(VectorXd &std) override;
	void generatePerturbation(VectorXd &perturbation) override;

#ifndef DISABLE_SPI
	bool supportsSPI() const override{ return n_channels == 1; }
#endif

protected:
	/**
	optional pointer to an illumination model
	if such a one is to be used as a photometric function
	*/
	AMParams::ILM ilm;

	/**
	multiplicative factor for the exponent in the likelihood
	*/
	double likelihood_alpha;

	VectorXdM I_diff;
	PixValT It_orig;
	ILMPixHessT ilm_d2f_dIt_type;	

	void cmptILMHessian(MatrixXd &d2f_dp2, const MatrixXd &dI_dpssm, 
		const double* I, const double* df_dg = nullptr);
	// functions to provide support for SPI
	virtual void getJacobian(RowVectorXd &jacobian, const bool *pix_mask,
		const RowVectorXd &curr_grad, const MatrixXd &pix_jacobian);
	virtual void getHessian(MatrixXd &hessian, const bool *pix_mask, const MatrixXd &pix_jacobian);
	virtual void getDifferenceOfJacobians(RowVectorXd &diff_of_jacobians, const bool *pix_mask,
		const MatrixXd &init_pix_jacobian, const MatrixXd &curr_pix_jacobian);
	virtual void getSumOfHessians(MatrixXd &sum_of_hessians, const bool *pix_mask,
		const MatrixXd &init_pix_jacobian, const MatrixXd &curr_pix_jacobian);
};

_MTF_END_NAMESPACE

#endif