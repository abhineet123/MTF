#ifndef MTF_PF_H
#define MTF_PF_H

#include "SearchMethod.h"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include "PFParams.h"

#ifdef ENABLE_PARALLEL
#include <memory>
#endif

_MTF_BEGIN_NAMESPACE

//! structure with all information about a particle
//! might be used for a possible future migration from 
//! the use of seperate structures for different properties
struct Particle{
	VectorXd state[2];
	VectorXd ar[2];
	double wt;
	double cum_wt;
	int curr_set_id;
	void resize(int state_size){
		state[0].resize(state_size);
		state[1].resize(state_size);
		ar[0].resize(state_size);
		ar[1].resize(state_size);
	}
};

// Particle Filter
template<class AM, class SSM>
class PF : public SearchMethod < AM, SSM > {

public:

	typedef PFParams ParamType;
	typedef ParamType::DynamicModel DynamicModel;
	typedef ParamType::UpdateType UpdateType;
	typedef ParamType::LikelihoodFunc LikelihoodFunc;
	typedef ParamType::ResamplingType ResamplingType;
	typedef ParamType::MeanType MeanType;

	typedef boost::mt11213b RandGenT;
	typedef boost::normal_distribution<double> MeasureDistT;
	//typedef boost::variate_generator<RandGenT&, MeasureDistT> MeasureGenT;
	typedef boost::random::uniform_real_distribution<double> ResampleDistT;
	//typedef boost::variate_generator<RandGenT&, ResampleDistT> ResampleGenT;
	typedef ResampleDistT::param_type ResampleDistParamT;

	using SearchMethod<AM, SSM> ::am;
	using SearchMethod<AM, SSM> ::ssm;
	using typename SearchMethod<AM, SSM> ::AMParams;
	using typename SearchMethod<AM, SSM> ::SSMParams;
	using SearchMethod<AM, SSM> ::cv_corners_mat;
	using SearchMethod<AM, SSM> ::name;
	using SearchMethod<AM, SSM> ::initialize;
	using SearchMethod<AM, SSM> ::update;

	PF(const ParamType *pf_params = nullptr,
		const AMParams *am_params = nullptr, const SSMParams *ssm_params = nullptr);
	~PF(){}

	void initialize(const cv::Mat &corners) override;
	void update() override;
	void setRegion(const cv::Mat& corners) override;

protected:

#ifdef ENABLE_PARALLEL
	typedef std::unique_ptr<AM> AMPTr;
	typedef std::unique_ptr<SSM> SSMPTr;
	std::vector<AMPTr> am_vec;
	std::vector<SSMPTr> ssm_vec;
#endif

	ParamType params;

	RandGenT measurement_gen;
	MeasureDistT measurement_dist;

	RandGenT resample_gen;
	ResampleDistT resample_dist;

	//! similarity of the initial patch (or template) with itself
	double max_similarity;

	int ssm_state_size;
	int frame_id;

	Matrix3d warp_update;

	CornersT mean_corners;
	CornersT prev_corners;

	VectorXd mean_state;
	// SSM states for all particles
	// 2 sets of particles are stored for efficient resampling
	std::vector<VectorXd> particle_states[2];
	// Update history for Auto Regression
	std::vector<VectorXd> particle_ar[2];
	int curr_set_id;

	//! ID of the particle with the maximum weight
	int max_wt_id;

	VectorXd particle_wts;
	VectorXd particle_cum_wts;

	VectorXd perturbed_state;
	VectorXd perturbed_ar;

	VectorXd state_sigma, state_mean;
	VectorXi resample_ids;
	VectorXd uniform_rand_nums;
	bool using_pix_sigma;

	double measurement_likelihood;
	double measurement_factor;
	cv::Mat curr_img_uchar;

	bool enable_adaptive_resampling;
	double min_eff_particles;

	char *log_fname;
	char *time_fname;

	void initializeParticles();
	void linearMultinomialResampling();
	void binaryMultinomialResampling();
	void residualResampling();
	void updateMeanCorners();
};

_MTF_END_NAMESPACE

#endif

