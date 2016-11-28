#include "mtf/AM/GB.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

//! value constructor
GBParams::GBParams(ILMParams *ilm_params, 
bool _additive_update) : ILMParams(ilm_params),
additive_update(_additive_update){}
//! default/copy constructor
GBParams::GBParams(const GBParams *params) : 
ILMParams(params),
additive_update(GB_ADDITIVE_UPDATE){
	if(params){
		additive_update = params->additive_update;
	}
}
GB::GB(const ParamType *_gb_params) :
IlluminationModel(_gb_params), params(_gb_params){
	name = "gb";
	printf("\n");
	printf("Using Gain and Bias illumination model with:\n");
	printf("additive_update: %d\n", params.additive_update);
}

void GB::apply(double *g, const double *I, const double *p){
	Map<VectorXd>(g, n_pix) = (1 + p[0]) * Map<const VectorXd>(I, n_pix).array() + p[1];
}
void GB::invert(double *inv_p, const double *p){
	if(params.additive_update){
		inv_p[0] = -p[0];
		inv_p[1] = -p[1];
	} else{
		inv_p[0] = -p[0] / (1 + p[0]);
		inv_p[1] = -p[1] / (1 + p[0]);
	}
}
void GB::update(double *new_p, const double *old_p, const double *dp){
	if(params.additive_update){
		new_p[0] = old_p[0] + dp[0];
		new_p[1] = old_p[1] + dp[1];
	} else{
		new_p[0] = old_p[0] + dp[0] * (old_p[0] + 1);
		new_p[1] = old_p[1] + dp[1] * (old_p[0] + 1);
	}
}
void GB::cmptParamJacobian(double *df_dp, const double *df_dg,
	const double *I, const double *p){
	if(params.additive_update){
		df_dp[0] = Map<const RowVectorXd>(df_dg, n_pix)*Map<const VectorXd>(I, n_pix);
		df_dp[1] = Map<const RowVectorXd>(df_dg, n_pix).sum();
	} else{
		df_dp[0] = (1 + p[0])*Map<const RowVectorXd>(df_dg, n_pix)*Map<const VectorXd>(I, n_pix);
		df_dp[1] = (1 + p[0])*Map<const RowVectorXd>(df_dg, n_pix).sum();
	}
}
void GB::cmptPixJacobian(double *df_dI, const double *df_dg,
	const double *I, const double *p){
	// dg_dI = (1 + p[0]) for both additive and compositional formulations
	Map<RowVectorXd>(df_dI, n_pix) = Map<const RowVectorXd>(df_dg, n_pix)*(1 + p[0]);
}
void GB::cmptParamHessian(double *_d2f_dp2, const double *d2f_dg2,
	const double *I, const double *p){
	if(d2f_dg2){
		MatrixXd temp_prod(2, n_pix);
		temp_prod.row(0) = Map<const VectorXd>(I, n_pix).transpose()*Map<const MatrixXd>(d2f_dg2, n_pix, n_pix);
		temp_prod.row(1) = Map<const MatrixXd>(d2f_dg2, n_pix, n_pix).colwise().sum();
		Map<Matrix2d>(_d2f_dp2).col(0) = temp_prod*Map<const VectorXd>(I, n_pix);
		Map<Matrix2d>(_d2f_dp2).col(1) = temp_prod.rowwise().sum();
		if(!params.additive_update){
			Map<Matrix2d>(_d2f_dp2) *= (1 + p[0]) * (1 + p[0]);
		}
	} else{
		// d2f_dg2 is identity
		Map<Matrix2d> d2f_dp2(_d2f_dp2);
		d2f_dp2(0, 0) =
			Map<const VectorXd>(I, n_pix).cwiseProduct(Map<const VectorXd>(I, n_pix)).sum();
		d2f_dp2(0, 1) = d2f_dp2(1, 0) =
			Map<const VectorXd>(I, n_pix).sum();
		d2f_dp2(1, 1) = n_pix;
		//utils::printMatrixToFile(d2f_dp2, "d2f_dp2", "log/fc_debug.txt");
		if(!params.additive_update){
			d2f_dp2 *= (1 + p[0]) * (1 + p[0]);
		}
	}
}
void GB::cmptPixHessian(double *d2f_dI2, const double *d2f_dg2,
	const double *I, const double *p){
	if(d2f_dg2){
		Map<MatrixXd>(d2f_dI2, n_pix, n_pix) = ((1 + p[0]) *  (1 + p[0])) * Map<const MatrixXd>(d2f_dg2, n_pix, n_pix);
	} else{
		*d2f_dI2 = (1 + p[0]) *  (1 + p[0]);
	}
}
void GB::cmptCrossHessian(double *d2f_dp_dI, const double *d2f_dg2,
	const double *I, const double *p){
	if(d2f_dg2){
	} else{
		double a_plus_1 = 1 + p[0];
		if(params.additive_update){
			Map<MatrixXd>(d2f_dp_dI, 2, n_pix).row(0) = a_plus_1 * Map<const RowVectorXd>(I, n_pix);
			Map<MatrixXd>(d2f_dp_dI, 2, n_pix).row(1).setConstant(a_plus_1);
		} else{
			double a_plus_1_squared = a_plus_1*a_plus_1;
			Map<MatrixXd>(d2f_dp_dI, 2, n_pix).row(0) = a_plus_1_squared * Map<const RowVectorXd>(I, n_pix);
			Map<MatrixXd>(d2f_dp_dI, 2, n_pix).row(1).setConstant(a_plus_1_squared);
		}
	}

}

void GB::cmptCrossHessian(double *d2f_dp_dI, const double *d2f_dg2,
	const double *df_dg, const double *I, const double *p){
	if(d2f_dg2){
	} else{
		double a_plus_1 = 1 + p[0];
		if(params.additive_update){
			Map<MatrixXd>(d2f_dp_dI, 2, n_pix).row(0) = a_plus_1 * Map<const RowVectorXd>(I, n_pix) + Map<const RowVectorXd>(df_dg, n_pix);
			Map<MatrixXd>(d2f_dp_dI, 2, n_pix).row(1).setConstant(a_plus_1);
		} else{
			double a_plus_1_squared = a_plus_1*a_plus_1;
			Map<MatrixXd>(d2f_dp_dI, 2, n_pix).row(0) = a_plus_1_squared * Map<const RowVectorXd>(I, n_pix) + (1 + p[0])*Map<const RowVectorXd>(df_dg, n_pix);
			Map<MatrixXd>(d2f_dp_dI, 2, n_pix).row(1).setConstant(a_plus_1_squared);
		}
	}

}

void GB::parseSamplerSigma(VectorXd &out_sigma, const VectorXd &in_sigma){
	assert(out_sigma.size() == 2);
	if(in_sigma.size() == 1){
		out_sigma.fill(in_sigma[0]);
	} else{
		if(in_sigma.size() != 2){
			throw std::invalid_argument(
				cv::format("GB::parseSamplerSigma :: sampler sigma has invalid size %d\n",
				in_sigma.size()));
		}
		out_sigma = in_sigma;
	}
}
void GB::parseSamplerMean(VectorXd &out_mean, const VectorXd &in_mean){
	assert(out_mean.size() == 2);
	if(in_mean.size() == 1){
		out_mean.fill(in_mean[0]);
	} else{
		if(in_mean.size() != 2){
			throw std::invalid_argument(
				cv::format("GB::parseSamplerMean :: sampler mean has invalid size %d\n",
				in_mean.size()));
		}
		out_mean = in_mean;
	}
}

_MTF_END_NAMESPACE




