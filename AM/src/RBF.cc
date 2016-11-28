#include "mtf/AM/RBF.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "mtf/Utilities/excpUtils.h"

_MTF_BEGIN_NAMESPACE

//! value constructor
RBFParams::RBFParams(ILMParams *ilm_params,
bool _additive_update,
int _n_ctrl_pts_x, int _n_ctrl_pts_y) :
ILMParams(ilm_params),
additive_update(_additive_update),
n_ctrl_pts_x(_n_ctrl_pts_x),
n_ctrl_pts_y(_n_ctrl_pts_y){}

//! default/copy constructor
RBFParams::RBFParams(const RBFParams *params) :
ILMParams(params),
additive_update(RBF_ADDITIVE_UPDATE),
n_ctrl_pts_x(RBF_N_CTRL_PTS),
n_ctrl_pts_y(RBF_N_CTRL_PTS){
	if(params){
		additive_update = params->additive_update;
		n_ctrl_pts_x = params->n_ctrl_pts_x;
		n_ctrl_pts_y = params->n_ctrl_pts_y;
	}
}

RBF::RBF(const ParamType *_pgb_params) :
IlluminationModel(_pgb_params), params(_pgb_params){
	name = "rbf";
	printf("\n");
	printf("Using Radial Basis Function illumination model with:\n");
	printf("sampling_res: %d x %d\n", resx, resy);
	printf("n_ctrl_pts: %d x %d\n", params.n_ctrl_pts_x, params.n_ctrl_pts_y);

	n_ctrl_pts = params.n_ctrl_pts_x*params.n_ctrl_pts_y;

	state_size = n_ctrl_pts + 1;
	bias_id = state_size - 1;

	printf("state_size: %d\n", state_size);

	region_size_x = resx / params.n_ctrl_pts_x;
	region_size_y = resy / params.n_ctrl_pts_y;

	if(region_size_x <= 0 || region_size_y <= 0){
		throw std::invalid_argument(
			cv::format("RBF :: Patch size : %d x %d is not enough to use the specified region spacing and / or count",
			resx, resy));
	}
	region_bounds_x.resize(params.n_ctrl_pts_x, Eigen::NoChange);
	region_bounds_y.resize(params.n_ctrl_pts_y, Eigen::NoChange);

	for(int idx = 0; idx < params.n_ctrl_pts_x; ++idx){
		region_bounds_x(idx, 0) = idx*region_size_x;
		region_bounds_x(idx, 1) = region_bounds_x(idx, 0) + region_size_x - 1;
	}
	region_bounds_x(params.n_ctrl_pts_x - 1, 1) = resx - 1;

	for(int idy = 0; idy < params.n_ctrl_pts_y; ++idy){
		region_bounds_y(idy, 0) = idy*region_size_y;
		region_bounds_y(idy, 1) = region_bounds_y(idy, 0) + region_size_y - 1;
	}
	region_bounds_y(params.n_ctrl_pts_x - 1, 1) = resy - 1;

	ctrl_pts_idx.resize(params.n_ctrl_pts_y, params.n_ctrl_pts_x);
	ctrl_pts.resize(n_ctrl_pts, Eigen::NoChange);
	for(int idy = 0; idy < params.n_ctrl_pts_y; ++idy){
		for(int idx = 0; idx < params.n_ctrl_pts_x; ++idx){
			ctrl_pts_idx(idy, idx) = idy * params.n_ctrl_pts_x + idx;
			ctrl_pts(ctrl_pts_idx(idy, idx), 0) = static_cast<double>(region_bounds_x(idx, 0) + region_bounds_x(idx, 1)) / 2.0;
			ctrl_pts(ctrl_pts_idx(idy, idx), 1) = static_cast<double>(region_bounds_y(idx, 0) + region_bounds_y(idx, 1)) / 2.0;
		}
	}
	pix_wts.resize(n_pix, n_ctrl_pts);
	int pix_id = 0;
	for(int y = 0; y < resy; ++y){
		for(int x = 0; x < resx; ++x){
			double wt_sum = 0;
			for(int pt_id = 0; pt_id < n_ctrl_pts; ++pt_id){
				double diff_x = x - ctrl_pts(pt_id, 0);
				double diff_y = y - ctrl_pts(pt_id, 1);
				pix_wts(pix_id, pt_id) = 1.0 / (1.0 + diff_x*diff_x + diff_y*diff_y);
				wt_sum += pix_wts(pix_id, pt_id);				
			}
			pix_wts.row(pix_id) /= wt_sum;
			++pix_id;
		}
	}
	printf("Only additive update is currently supported\n");
}

void RBF::apply(double *g, const double *I, const double *p){
	double bias = p[bias_id];
	for(int pix_id = 0; pix_id < n_pix; ++pix_id){
		double gain = 1;
		for(int pt_id = 0; pt_id < n_ctrl_pts; ++pt_id){
			gain += pix_wts(pix_id, pt_id)*p[pt_id];
		}
		g[pix_id] = gain * I[pix_id] + bias;
	}
	//showSubRegions(g);
}
void RBF::invert(double *inv_p, const double *p){
	for(int state_id = 0; state_id < state_size; ++state_id){
		inv_p[state_id] = -p[state_id];
	}
}
void RBF::update(double *new_p, const double *old_p, const double *dp){
	for(int state_id = 0; state_id < state_size; ++state_id){
		new_p[state_id] = old_p[state_id] + dp[state_id];
	}
}
void RBF::cmptParamJacobian(double *df_dp, const double *df_dg,
	const double *I, const double *p){
	for(int pt_id = 0; pt_id < n_ctrl_pts; ++pt_id){
		df_dp[pt_id] = 0;
		for(int pix_id = 0; pix_id < n_pix; ++pix_id){
			df_dp[pt_id] += pix_wts(pix_id, pt_id)*I[pix_id] * df_dg[pix_id];
		}
	}
	df_dp[bias_id] = 0;
	for(int pix_id = 0; pix_id < n_pix; ++pix_id){
		df_dp[bias_id] += df_dg[pix_id];
	}
}
void RBF::cmptPixJacobian(double *df_dI, const double *df_dg,
	const double *I, const double *p){
	for(int pix_id = 0; pix_id < n_pix; ++pix_id){
		double gain = 1;
		for(int pt_id = 0; pt_id < n_ctrl_pts; ++pt_id){
			gain += pix_wts(pix_id, pt_id)*p[pt_id];
		}
		df_dI[pix_id] = df_dg[pix_id] * gain;
	}

}
void RBF::cmptParamHessian(double *_d2f_dp2, const double *_d2f_dg2,
	const double *I, const double *p){

	switch(d2f_dg2_type){
	case  PixHessType::Constant:
	{		
		//! d2f_dg2 is a constant multiple of identity matrix	
		double d2f_dg2_const = _d2f_dg2 ? *_d2f_dg2 : 1;
		MatrixXd dg_dp(n_pix, state_size);
		dg_dp.leftCols(state_size - 1) = pix_wts.array().colwise()*Map<const VectorXd>(I, n_pix).array();
		dg_dp.col(state_size - 1).fill(1);
		Map<MatrixXd>(_d2f_dp2, state_size, state_size) = d2f_dg2_const * dg_dp.transpose()*dg_dp;
		break;
	}
	case  PixHessType::Diagonal:
	{
		throw utils::FunctonNotImplemented("RBF::cmptParamHessian not implemented for diagonal d2f_dg2 yet");

	}
	case  PixHessType::General:
	{
		throw utils::FunctonNotImplemented("RBF::cmptParamHessian not implemented for diagonal d2f_dg2 yet");

	}
	default:
		throw std::invalid_argument(
			cv::format("RBF::cmptParamHessian :: Invalid hessian type provided: %d", d2f_dg2_type));
	}
}
void RBF::cmptPixHessian(double *_d2f_dI2, const double *_d2f_dg2,
	const double *I, const double *p){
	switch(d2f_dg2_type){
	case  PixHessType::Constant:
	{
		Map<VectorXd> d2f_dI2(_d2f_dI2, n_pix);
		double d2f_dg2_const = _d2f_dg2 ? *_d2f_dg2 : 1;
		for(int pix_id = 0; pix_id < n_pix; ++pix_id){
			double gain = 1;
			for(int pt_id = 0; pt_id < n_ctrl_pts; ++pt_id){
				gain += pix_wts(pix_id, pt_id)*p[pt_id];
			}
			d2f_dI2[pix_id] = gain*gain*d2f_dg2_const;

		}
		break;
	}
	case  PixHessType::Diagonal:
	{
		throw utils::FunctonNotImplemented("RBF::cmptPixHessian not implemented for diagonal d2f_dg2 yet");
	}
	case  PixHessType::General:
	{
		throw utils::FunctonNotImplemented("RBF::cmptPixHessian not implemented for general d2f_dg2 yet");

	}
	default:
		throw std::invalid_argument(
			cv::format("RBF::cmptPixHessian :: Invalid hessian type provided: %d", d2f_dg2_type));
	}
}
void RBF::cmptCrossHessian(double *_d2f_dp_dI, const double *d2f_dg2,
	const double *I, const double *p){
	if(d2f_dg2){
	} else{
		//! d2f_dg2 is identity
		Map<MatrixXd> d2f_dp_dI(_d2f_dp_dI, state_size, n_pix);
		for(int pix_id = 0; pix_id < n_pix; ++pix_id){
			double gain = 1;
			for(int pt_id = 0; pt_id < n_ctrl_pts; ++pt_id){
				gain += pix_wts(pix_id, pt_id)*p[pt_id];
			}
			for(int pt_id = 0; pt_id < n_ctrl_pts; ++pt_id){
				d2f_dp_dI(pt_id, pix_id) = (gain*I[pix_id]) * pix_wts(pix_id, pt_id);
			}
			d2f_dp_dI(bias_id, pix_id) = gain;
		}
	}

}

void RBF::cmptCrossHessian(double *_d2f_dp_dI, const double *d2f_dg2,
	const double *df_dg, const double *I, const double *p){
	if(d2f_dg2){
	} else{
		//! d2f_dg2 is identity
		Map<MatrixXd> d2f_dp_dI(_d2f_dp_dI, state_size, n_pix);
		for(int pix_id = 0; pix_id < n_pix; ++pix_id){
			double gain = 1;
			for(int pt_id = 0; pt_id < n_ctrl_pts; ++pt_id){
				gain += pix_wts(pix_id, pt_id)*p[pt_id];
			}
			for(int pt_id = 0; pt_id < n_ctrl_pts; ++pt_id){
				d2f_dp_dI(pt_id, pix_id) = (gain*I[pix_id] + df_dg[pix_id]) * pix_wts(pix_id, pt_id);
			}
			d2f_dp_dI(bias_id, pix_id) = gain;
		}
	}

}

IlluminationModel::PixHessType RBF::getPixHessType(){
	switch(d2f_dg2_type){
	case  PixHessType::Constant:
		return  PixHessType::Diagonal;
	case  PixHessType::Diagonal:
		return  PixHessType::General;
	case  PixHessType::General:
		return  PixHessType::General;
	default:
		throw std::invalid_argument(
			cv::format("RBF:: d2f_dg2 has nvalid type : %d", d2f_dg2_type));
	}
}

void RBF::parseSamplerSigma(VectorXd &out_sigma, const VectorXd &in_sigma){
	assert(out_sigma.size() == state_size);
	if(in_sigma.size() == 1){
		out_sigma.fill(in_sigma[0]);
	} else if(in_sigma.size() == 2){
		out_sigma.head(state_size - 1).fill(in_sigma[0]);
		out_sigma[state_size - 1] = in_sigma[1];
	} else{
		if(in_sigma.size() != state_size){
			throw std::invalid_argument(
				cv::format("RBF::parseSamplerSigma :: sampler sigma has invalid size %d which should be %d\n",
				in_sigma.size(), state_size));
		}
		out_sigma = in_sigma;
	}
}
void RBF::parseSamplerMean(VectorXd &out_mean, const VectorXd &in_mean){
	assert(out_mean.size() == state_size);
	if(in_mean.size() == 1){
		out_mean.fill(in_mean[0]);
	} else if(in_mean.size() == 2){
		out_mean.head(state_size - 1).fill(in_mean[0]);
		out_mean[state_size - 1] = in_mean[1];
	} else{
		if(in_mean.size() != state_size){
			throw std::invalid_argument(
				cv::format("RBF::parseSamplerMean :: sampler mean has invalid size %d which should be %d\n",
				in_mean.size(), state_size));
		}
		out_mean = in_mean;
	}
}

_MTF_END_NAMESPACE




