#include "mtf/AM/SSDBase.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/spiUtils.h"

#include <boost/random/random_device.hpp>
#include <boost/random/seed_seq.hpp>

_MTF_BEGIN_NAMESPACE

SSDBase::SSDBase(const AMParams *am_params, const int _n_channels) :
AppearanceModel(am_params, _n_channels), ilm(nullptr), likelihood_alpha(1),
I_diff(0, 0){
	if(am_params){
		ilm = am_params->ilm;
		likelihood_alpha = am_params->likelihood_alpha;
	}
	if(ilm){
		ilm->setPixHessType(ILMPixHessT::Constant);
		ilm_d2f_dIt_type = ilm->getPixHessType();
		state_size = ilm->getStateSize();
		if(ilm_d2f_dIt_type == ILMPixHessT::General){
			d2f_dIt2.resize(n_pix, n_pix);
		}
	}
	pix_norm_mult = 1;
	pix_norm_add = 0;
}

void SSDBase::initializeSimilarity(){
	if(is_initialized.similarity)
		return;

	df_dI0.resize(patch_size);
	new (&I_diff) VectorXdM(df_dI0.data(), patch_size);
	I_diff.fill(0);
	f = 0;
	if(ilm){
		It_orig.resize(patch_size);
		It_orig = It;
		p_am.resize(state_size);
		ilm->initialize(p_am.data());
	}
	is_initialized.similarity = true;
}

void SSDBase::initializeGrad(){
	if(is_initialized.grad)
		return;

	df_dIt.resize(patch_size);
	df_dIt = df_dI0;

	if(ilm){
		df_dpam.resize(state_size);
		df_dgt.resize(patch_size);
		df_dg0.resize(patch_size);
		df_dg0 = df_dI0;
		df_dgt = df_dIt;
		ilm->cmptParamJacobian(df_dpam.data(), df_dgt.data(), It_orig.data(), p_am.data());
		ilm->cmptPixJacobian(df_dIt.data(), df_dgt.data(), It_orig.data(), p_am.data());
	}
	is_initialized.grad = true;
}
double SSDBase::getLikelihood() const{
	/**
	since SSD can be numerically too large for exponential function to work,
	we take the square root of the per pixel SSD instead;
	it is further normalized by dividing with the range of pixel values to avoid
	very small numbers that may lead to loss of information due to the limited precision
	of floating point operations;
	*/
	return exp(-likelihood_alpha * sqrt(-f / (static_cast<double>(patch_size))));
}

void SSDBase::updateSimilarity(bool prereq_only){
	if(ilm){
		//! I_t actually refers to g(I_t, p_am) to keep the illumination model transparent
		//! and avoid any unnecessary runtime costs if it is not used
		It_orig = It;
		ilm->apply(It.data(), It_orig.data(), p_am.data());
	}
	I_diff = It - I0;
	if(prereq_only){ return; }
#ifndef DISABLE_SPI
	if(spi_mask){
		VectorXd masked_err_vec = Map<const VectorXb>(spi_mask, n_pix).select(I_diff, 0);
		f = -masked_err_vec.squaredNorm() / 2;
	} else{
#endif
		f = -I_diff.squaredNorm() / 2;
#ifndef DISABLE_SPI
	}
#endif
	//utils::printMatrix(p_am.transpose(), "p_am");
	//printf("f: %f\n", f);
}
void SSDBase::updateState(const VectorXd& state_update){
	if(ilm){
		assert(state_update.size() == state_size);
		VectorXd p_am_old = p_am;
		ilm->update(p_am.data(), p_am_old.data(), state_update.data());
		//utils::printMatrix(state_update.transpose(), "state_update");
		//utils::printMatrix(p_am_old.transpose(), "p_am before update");
		//utils::printMatrix(p_am.transpose(), "p_am after update");
	}
}
void SSDBase::invertState(VectorXd& inv_p, const VectorXd& p){
	if(ilm){
		assert(inv_p.size() == state_size && p.size() == state_size);
		ilm->invert(inv_p.data(), p.data());
	}	
}

// curr_grad is same as negative of init_grad
void SSDBase::updateCurrGrad(){
	df_dIt = -df_dI0;
	if(ilm){
		df_dgt = df_dIt;
		ilm->cmptPixJacobian(df_dIt.data(), df_dgt.data(), It_orig.data(), p_am.data());
	}
}

void SSDBase::cmptInitJacobian(RowVectorXd &df_dp,
	const MatrixXd &dI0_dpssm){
	assert(df_dp.size() == state_size + dI0_dpssm.cols());
	assert(dI0_dpssm.rows() == patch_size);
	if(ilm){
		df_dp.head(dI0_dpssm.cols()).noalias() = df_dI0 * dI0_dpssm;
		ilm->cmptParamJacobian(df_dpam.data(), df_dg0.data(), 
			I0.data(), p_am.data());
		df_dp.tail(state_size) = df_dpam;
	} else{
#ifndef DISABLE_SPI
		if(spi_mask){
			getJacobian(df_dp, spi_mask, df_dI0, dI0_dpssm);
		} else{
#endif
			df_dp.noalias() = df_dI0 * dI0_dpssm;
#ifndef DISABLE_SPI
		}
#endif
	}
}
void SSDBase::cmptCurrJacobian(RowVectorXd &df_dp,
	const MatrixXd &dIt_dpssm){
	assert(df_dp.size() == state_size + dIt_dpssm.cols());
	assert(dIt_dpssm.rows() == patch_size);

	if(ilm){
		df_dp.head(dIt_dpssm.cols()).noalias() = df_dIt * dIt_dpssm;
		ilm->cmptParamJacobian(df_dpam.data(), df_dgt.data(), 
			It_orig.data(), p_am.data());
		df_dp.tail(state_size) = df_dpam;
	} else{
#ifndef DISABLE_SPI
		if(spi_mask){
			getJacobian(df_dp, spi_mask, df_dIt, dIt_dpssm);
		} else{
#endif
			//printf("df_dp: %ld x %ld\n", df_dp.rows(), df_dp.cols());
			//printf("df_dIt: %ld x %ld\n", df_dIt.rows(), df_dIt.cols());
			//printf("dIt_dpssm: %ld x %ld\n", dIt_dpssm.rows(), dIt_dpssm.cols());
			df_dp.noalias() = df_dIt * dIt_dpssm;
#ifndef DISABLE_SPI
		}
#endif
	}
}
void SSDBase::cmptDifferenceOfJacobians(RowVectorXd &df_dp_diff,
	const MatrixXd &dI0_dpssm, const MatrixXd &dIt_dpssm){
	assert(df_dp_diff.size() == state_size + dIt_dpssm.cols());
	assert(dI0_dpssm.cols() == dIt_dpssm.cols());
	assert(dIt_dpssm.rows() == patch_size && dI0_dpssm.rows() == patch_size);
	if(ilm){
		df_dp_diff.head(dIt_dpssm.cols()).noalias() = df_dIt * (dI0_dpssm + dIt_dpssm);
		VectorXd df_dpam_0(state_size), df_dpam_t(state_size);
		ilm->cmptParamJacobian(df_dpam_0.data(), df_dg0.data(), I0.data(), p_am.data());
		ilm->cmptParamJacobian(df_dpam_t.data(), df_dgt.data(), It_orig.data(), p_am.data());
		df_dp_diff.tail(state_size) = df_dpam_t - df_dpam_0;
	} else{
#ifndef DISABLE_SPI
		if(spi_mask){
			getDifferenceOfJacobians(df_dp_diff, spi_mask, dI0_dpssm, dIt_dpssm);
		} else{
#endif
			df_dp_diff.noalias() = df_dIt * (dI0_dpssm + dIt_dpssm);
#ifndef DISABLE_SPI
		}
#endif
	}
}

void SSDBase::cmptILMHessian(MatrixXd &d2f_dp2, const MatrixXd &dI_dpssm,
	const double* I, const double* df_dg){
	int ssm_state_size = dI_dpssm.cols();

	assert(d2f_dp2.rows() == state_size + ssm_state_size);
	assert(d2f_dp2.cols() == state_size + ssm_state_size);

	MatrixXd d2f_dpam2(state_size, state_size);
	ilm->cmptParamHessian(d2f_dpam2.data(), nullptr, df_dg, I, p_am.data());
	d2f_dp2.bottomRightCorner(state_size, state_size) = -d2f_dpam2;
	switch(ilm_d2f_dIt_type){
	case  ILMPixHessT::Constant:
	{
		double d2f_dIt2_const;
		if(df_dg){
			ilm->cmptPixHessian(&d2f_dIt2_const, nullptr, df_dg, I, p_am.data());
		} else{
			ilm->cmptPixHessian(&d2f_dIt2_const, nullptr, I, p_am.data());
		}
		
		d2f_dp2.topLeftCorner(ssm_state_size, ssm_state_size).noalias() = -d2f_dIt2_const * dI_dpssm.transpose() * dI_dpssm;
		break;
	}
	case  ILMPixHessT::Diagonal:
	{
		VectorXd d2f_dIt2_diag(patch_size);
		if(df_dg){
			ilm->cmptPixHessian(d2f_dIt2_diag.data(), nullptr, df_dg, I, p_am.data());
		} else{
			ilm->cmptPixHessian(d2f_dIt2_diag.data(), nullptr, I, p_am.data());
		}
		d2f_dp2.topLeftCorner(ssm_state_size, ssm_state_size).noalias() =
			-(dI_dpssm.array().colwise() * d2f_dIt2_diag.array()).matrix().transpose() * dI_dpssm;
		break;
	}
	case  ILMPixHessT::General:
	{
		if(df_dg){
			ilm->cmptPixHessian(d2f_dIt2.data(), nullptr, df_dg, I, p_am.data());
		} else{
			ilm->cmptPixHessian(d2f_dIt2.data(), nullptr, I, p_am.data());
		}
		d2f_dp2.topLeftCorner(ssm_state_size, ssm_state_size).noalias() = -dI_dpssm.transpose() * d2f_dIt2 * dI_dpssm;
		break;
	}
	default:
		throw utils::InvalidArgument(
			cv::format("SSDBase :: ILM has invalid hessian type provided: %d", ilm_d2f_dIt_type));
	}
	if(df_dg){
		ilm->cmptCrossHessian(d2f_dpam_dIt.data(), nullptr, df_dg, I, p_am.data());
	} else{
		ilm->cmptCrossHessian(d2f_dpam_dIt.data(), nullptr, I, p_am.data());
	}
	d2f_dp2.topRightCorner(ssm_state_size, state_size).transpose() =
		d2f_dp2.bottomLeftCorner(state_size, ssm_state_size) = -d2f_dpam_dIt*dI_dpssm;
}

void SSDBase::cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dpssm){
	assert(d2f_dp2.rows() == dI0_dpssm.cols() + state_size && d2f_dp2.rows() == d2f_dp2.cols());
	assert(dI0_dpssm.rows() == patch_size);
	if(ilm){
		cmptILMHessian(d2f_dp2, dI0_dpssm, I0.data());
	} else{
#ifndef DISABLE_SPI
		if(spi_mask){
			getHessian(d2f_dp2, spi_mask, dI0_dpssm);
		} else{
#endif
			d2f_dp2.noalias() = -dI0_dpssm.transpose() * dI0_dpssm;
#ifndef DISABLE_SPI
		}
#endif
	}
}
void SSDBase::cmptCurrHessian(MatrixXd &d2f_dp2,
	const MatrixXd &dIt_dpssm){
	assert(d2f_dp2.rows() == dIt_dpssm.cols() + state_size && d2f_dp2.rows() == d2f_dp2.cols());
	assert(dIt_dpssm.rows() == patch_size);
	if(ilm){
		cmptILMHessian(d2f_dp2, dIt_dpssm, It_orig.data());
	} else{
#ifndef DISABLE_SPI
		if(spi_mask){
			getHessian(d2f_dp2, spi_mask, dIt_dpssm);
		} else{
#endif
			d2f_dp2.noalias() = -dIt_dpssm.transpose() * dIt_dpssm;
#ifndef DISABLE_SPI
		}
#endif
	}
}
//! analogous to cmptDifferenceOfJacobians except for computing the difference between the current and initial Hessians
void SSDBase::cmptSumOfHessians(MatrixXd &d2f_dp2_sum,
	const MatrixXd &dI0_dpssm,	const MatrixXd &dIt_dpssm){
	assert(d2f_dp2_sum.cols() == d2f_dp2_sum.rows());
	assert(dI0_dpssm.rows() == patch_size && dIt_dpssm.rows() == patch_size);

	if(ilm){
		MatrixXd d2f_dp2t(d2f_dp2_sum.rows(), d2f_dp2_sum.cols());
		cmptILMHessian(d2f_dp2t, dIt_dpssm, It_orig.data());
		MatrixXd d2f_dp20(d2f_dp2_sum.rows(), d2f_dp2_sum.cols());
		cmptILMHessian(d2f_dp20, dI0_dpssm, I0.data());
		d2f_dp2_sum.noalias() = d2f_dp2t + d2f_dp20;
	} else{
#ifndef DISABLE_SPI
		if(spi_mask){
			getSumOfHessians(d2f_dp2_sum, spi_mask,
				dI0_dpssm, dIt_dpssm);
		} else{
#endif
			d2f_dp2_sum.noalias() = -(dI0_dpssm.transpose() * dI0_dpssm
				+ dIt_dpssm.transpose() * dIt_dpssm);
#ifndef DISABLE_SPI
		}
#endif
	}
}

void SSDBase::cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dpssm,
	const MatrixXd &d2I0_dpssm2){
	int ssm_state_size = d2f_dp2.rows();
	assert(d2f_dp2.cols() == ssm_state_size + state_size);
	assert(d2I0_dpssm2.rows() == ssm_state_size * ssm_state_size && d2I0_dpssm2.cols() == n_channels*n_pix);

	assert(d2f_dp2.rows() == dI0_dpssm.cols() + state_size && d2f_dp2.rows() == d2f_dp2.cols());
	assert(dI0_dpssm.rows() == patch_size);
	if(ilm){
		cmptILMHessian(d2f_dp2, dI0_dpssm, I0.data(), df_dg0.data());
	} else{
#ifndef DISABLE_SPI
		if(spi_mask){
			getHessian(d2f_dp2, spi_mask, dI0_dpssm);
		} else{
#endif
			d2f_dp2.noalias() = -dI0_dpssm.transpose() * dI0_dpssm;
#ifndef DISABLE_SPI
		}
#endif
	}
	int ch_pix_id = 0;
	for(unsigned int pix_id = 0; pix_id < n_pix; pix_id++){
		spi_pt_check_mc(spi_mask, pix_id, ch_pix_id);
		for(unsigned int channel_id = 0; channel_id < n_channels; ++channel_id){
			d2f_dp2 += Map<const MatrixXd>(d2I0_dpssm2.col(ch_pix_id).data(), ssm_state_size, ssm_state_size) * df_dI0(ch_pix_id);
			++ch_pix_id;
		}
	}
}

void SSDBase::cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dpssm,
	const MatrixXd &d2It_dpssm2){
	int ssm_state_size = d2f_dp2.rows();

	assert(d2f_dp2.cols() == ssm_state_size + state_size);
	assert(d2It_dpssm2.rows() == ssm_state_size * ssm_state_size && d2It_dpssm2.cols() == n_channels * n_pix);

	assert(d2f_dp2.rows() == dIt_dpssm.cols() + state_size && d2f_dp2.rows() == d2f_dp2.cols());
	assert(dIt_dpssm.rows() == patch_size);
	if(ilm){
		cmptILMHessian(d2f_dp2, dIt_dpssm, It_orig.data(), df_dgt.data());
	} else{
#ifndef DISABLE_SPI
		if(spi_mask){
			getHessian(d2f_dp2, spi_mask, dIt_dpssm);
		} else{
#endif
			d2f_dp2.noalias() = -dIt_dpssm.transpose() * dIt_dpssm;
#ifndef DISABLE_SPI
		}
#endif
	}
	int ch_pix_id = 0;
	for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id){
		spi_pt_check_mc(spi_mask, pix_id, ch_pix_id);
		for(unsigned int channel_id = 0; channel_id < n_channels; ++channel_id){
			d2f_dp2 += Map<const MatrixXd>(d2It_dpssm2.col(ch_pix_id).data(), ssm_state_size, ssm_state_size) * df_dIt(ch_pix_id);
			++ch_pix_id;
		}
	}
}

void SSDBase::cmptSumOfHessians(MatrixXd &d2f_dp2_sum,
	const MatrixXd &dI0_dpssm, const MatrixXd &dIt_dpssm,
	const MatrixXd &d2I0_dpssm2, const MatrixXd &d2It_dpssm2){

	int ssm_state_size = d2f_dp2_sum.rows();
	assert(d2f_dp2_sum.cols() == ssm_state_size);
	assert(d2I0_dpssm2.rows() == ssm_state_size * ssm_state_size && d2I0_dpssm2.cols() == n_channels * n_pix);
	assert(d2It_dpssm2.rows() == ssm_state_size * ssm_state_size && d2It_dpssm2.cols() == n_channels * n_pix);

	if(ilm){
		MatrixXd d2f_dp2t(d2f_dp2_sum.rows(), d2f_dp2_sum.cols());
		cmptILMHessian(d2f_dp2t, dIt_dpssm, It_orig.data(), df_dgt.data());
		MatrixXd d2f_dp20(d2f_dp2_sum.rows(), d2f_dp2_sum.cols());
		cmptILMHessian(d2f_dp20, dI0_dpssm, I0.data(), df_dgt.data());
		d2f_dp2_sum.noalias() = d2f_dp2t + d2f_dp20;
	} else{
#ifndef DISABLE_SPI
		if(spi_mask){
			getSumOfHessians(d2f_dp2_sum, spi_mask,
				dI0_dpssm, dIt_dpssm);
		} else{
#endif
			d2f_dp2_sum.noalias() = -(dI0_dpssm.transpose() * dI0_dpssm
				+ dIt_dpssm.transpose() * dIt_dpssm);
#ifndef DISABLE_SPI
		}
#endif
	}

	int ch_pix_id = 0;
	for(unsigned int pix_id = 0; pix_id < n_pix; pix_id++){
		spi_pt_check_mc(spi_mask, pix_id, ch_pix_id);
		for(unsigned int channel_id = 0; channel_id < n_channels; ++channel_id){
			d2f_dp2_sum += df_dI0(pix_id)*
				(Map<const MatrixXd>(d2I0_dpssm2.col(ch_pix_id).data(), ssm_state_size, ssm_state_size)
				+ Map<const MatrixXd>(d2It_dpssm2.col(ch_pix_id).data(), ssm_state_size, ssm_state_size));
			++ch_pix_id;
		}
	}
}

void SSDBase::getJacobian(RowVectorXd &jacobian, const bool *pix_mask,
	const RowVectorXd &df_dI, const MatrixXd &dI_dpssm){
	assert(dI_dpssm.rows() == patch_size && dI_dpssm.rows() == jacobian.size());
	jacobian.setZero();
	int ch_pix_id = 0;
	for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id){
		spi_check_mc(pix_mask, pix_id, ch_pix_id);
		for(unsigned int channel_id = 0; channel_id < n_channels; ++channel_id){
			jacobian += df_dI[ch_pix_id] * dI_dpssm.row(ch_pix_id);
			++ch_pix_id;
		}

	}
}

void SSDBase::getDifferenceOfJacobians(RowVectorXd &diff_of_jacobians, const bool *pix_mask,
	const MatrixXd &dI0_dpssm, const MatrixXd &dIt_dpssm){
	assert(dI0_dpssm.rows() == n_channels * n_pix && dIt_dpssm.rows() == n_channels * n_pix);
	assert(dI0_dpssm.rows() == diff_of_jacobians.size());

	diff_of_jacobians.setZero();
	int ch_pix_id = 0;
	for(unsigned int pix_id = 0; pix_id < n_pix; pix_id++){
		spi_check_mc(pix_mask, pix_id, ch_pix_id);
		for(unsigned int channel_id = 0; channel_id < n_channels; ++channel_id){
			diff_of_jacobians += df_dIt[pix_id] *
				(dI0_dpssm.row(ch_pix_id) + dIt_dpssm.row(ch_pix_id));
			++ch_pix_id;
		}

	}
}

void SSDBase::getHessian(MatrixXd &d2f_dp2, const bool *pix_mask, const MatrixXd &dI_dpssm){
	assert(dI_dpssm.rows() == n_channels * n_pix);

	d2f_dp2.setZero();
	int ch_pix_id = 0;
	for(unsigned int pix_id = 0; pix_id < n_pix; pix_id++){
		spi_check_mc(pix_mask, pix_id, ch_pix_id);
		for(unsigned int channel_id = 0; channel_id < n_channels; ++channel_id){
			d2f_dp2 -= dI_dpssm.row(ch_pix_id).transpose()*dI_dpssm.row(ch_pix_id);
			++ch_pix_id;
		}
	}
}

void SSDBase::getSumOfHessians(MatrixXd &d2f_dp2, const bool *pix_mask,
	const MatrixXd &dI0_dpssm, const MatrixXd &dIt_dpssm){
	assert(dI0_dpssm.rows() == n_channels * n_pix && dIt_dpssm.rows() == n_channels * n_pix);
	assert(d2f_dp2.rows() == d2f_dp2.cols() && d2f_dp2.rows() == dI0_dpssm.cols());

	d2f_dp2.setZero();
	unsigned int ch_pix_id = 0;
	for(unsigned int pix_id = 0; pix_id < n_pix; pix_id++){
		spi_check_mc(pix_mask, pix_id, ch_pix_id);
		for(unsigned int channel_id = 0; channel_id < n_channels; ++channel_id){
			d2f_dp2 -= dI0_dpssm.row(ch_pix_id).transpose()*dI0_dpssm.row(ch_pix_id)
				+ dIt_dpssm.row(ch_pix_id).transpose()*dIt_dpssm.row(ch_pix_id);
			++ch_pix_id;
		}
	}
}

// -------------------------------------------------------------------------- //
// --------------------------- Stochastic Sampler --------------------------- //
// -------------------------------------------------------------------------- //

void SSDBase::initializeSampler(const VectorXd &_state_sigma,
	const VectorXd &_state_mean){
	if(!ilm){ return; }

	VectorXd state_sigma(state_size), state_mean(state_size);
	ilm->parseSamplerSigma(state_sigma, _state_sigma);
	ilm->parseSamplerMean(state_mean, _state_mean);

	printf("Initializing %s sampler with sigma: ", 
		ilm ? (name + "/" + ilm->name).c_str() : name.c_str());
	utils::printMatrix(state_sigma.transpose(), nullptr, "%e");

	state_perturbation.resize(state_size);
	rand_gen.resize(state_size);
	rand_dist.resize(state_size);

	boost::random_device r;
	for(int state_id = 0; state_id < state_size; state_id++) {
		boost::random::seed_seq seed{ r(), r(), r(), r(), r(), r(), r(), r() };
		rand_gen[state_id] = SampleGenT(seed);
		rand_dist[state_id] = SampleDistT(state_mean[state_id], state_sigma[state_id]);
	}
	is_initialized.sampler = true;
}


void SSDBase::setSampler(const VectorXd &_state_sigma,
	const VectorXd &_state_mean){
	if(!ilm){ return; }
	VectorXd state_sigma(state_size), state_mean(state_size);
	ilm->parseSamplerSigma(state_sigma, _state_sigma);
	ilm->parseSamplerMean(state_mean, _state_mean);
	for(int state_id = 0; state_id < state_size; state_id++){
		rand_dist[state_id].param(DistParamT(state_mean[state_id], state_sigma[state_id]));
	}
}

void SSDBase::setSamplerMean(const VectorXd &_state_mean){
	if(!ilm){ return; }
	VectorXd state_mean(state_size);
	ilm->parseSamplerMean(state_mean, _state_mean);
	for(int state_id = 0; state_id < state_size; state_id++){
		double state_sigma = rand_dist[state_id].sigma();
		rand_dist[state_id].param(DistParamT(state_mean[state_id], state_sigma));
	}
}
void SSDBase::setSamplerSigma(const VectorXd &_state_sigma){
	VectorXd state_sigma(state_size);
	ilm->parseSamplerSigma(state_sigma, _state_sigma);
	for(int state_id = 0; state_id < state_size; state_id++){
		double mean = rand_dist[state_id].mean();
		rand_dist[state_id].param(DistParamT(mean, state_sigma[state_id]));
	}
}

void SSDBase::getSamplerSigma(VectorXd &std){
	if(!ilm){ return; }
	assert(std.size() == state_size);
	for(int state_id = 0; state_id < state_size; state_id++){
		std(state_id) = rand_dist[state_id].sigma();
	}
}
void SSDBase::getSamplerMean(VectorXd &mean){
	if(!ilm){ return; }
	assert(mean.size() == state_size);
	for(int state_id = 0; state_id < state_size; state_id++){
		mean(state_id) = rand_dist[state_id].mean();
	}
}

void SSDBase::generatePerturbation(VectorXd &perturbation){
	if(!ilm){ return; }
	assert(perturbation.size() == state_size);
	for(int state_id = 0; state_id < state_size; state_id++){
		perturbation(state_id) = rand_dist[state_id](rand_gen[state_id]);
	}
}


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
double SSDBaseDist::operator()(const double* a, const double* b,
	size_t size, double worst_dist) const{
	double result = 0;
	double diff0, diff1, diff2, diff3;
	const double* last = a + size;
	const double* lastgroup = last - 3;

	/* Process 4 items with each loop for efficiency. */
	while(a < lastgroup){
		diff0 = (a[0] - b[0]);
		diff1 = (a[1] - b[1]);
		diff2 = (a[2] - b[2]);
		diff3 = (a[3] - b[3]);
		result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
		a += 4;
		b += 4;

		if((worst_dist > 0) && (result > worst_dist)){
			return result;
		}
	}
	/* Process last 0-3 pixels.  Not needed for standard vector lengths. */
	while(a < last){
		diff0 = (*a++ - *b++);
		result += diff0 * diff0;
	}
	return result;
}


_MTF_END_NAMESPACE

