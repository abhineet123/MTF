#include "mtf/AM/PGB.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

_MTF_BEGIN_NAMESPACE

//! value constructor
PGBParams::PGBParams(ILMParams *ilm_params,
bool _additive_update,
int _n_regions_x, int _n_regions_y) :
ILMParams(ilm_params),
additive_update(_additive_update),
n_regions_x(_n_regions_x),
n_regions_y(_n_regions_y){}

//! default/copy constructor
PGBParams::PGBParams(const PGBParams *params) :
ILMParams(params),
additive_update(PGB_ADDITIVE_UPDATE),
n_regions_x(PGB_N_REGIONS),
n_regions_y(PGB_N_REGIONS){
	if(params){
		additive_update = params->additive_update;
		n_regions_x = params->n_regions_x;
		n_regions_y = params->n_regions_y;
	}
}

PGB::PGB(const ParamType *_pgb_params) :
IlluminationModel(_pgb_params), params(_pgb_params){
	name = "pgb";
	printf("\n");
	printf("Using Piecewise Gain and Bias ILM with:\n");
	printf("sampling_res: %d x %d\n", resx, resy);
	printf("sub_regions: %d x %d\n", params.n_regions_x, params.n_regions_y);

	n_regions = params.n_regions_x*params.n_regions_y;

	state_size = n_regions + 1;
	bias_id = state_size - 1;

	printf("state_size: %d\n", state_size);

	region_size_x = resx / params.n_regions_x;
	region_size_y = resy / params.n_regions_y;

	if(region_size_x <= 0 || region_size_y <= 0){
		throw std::invalid_argument(
			cv::format("PGB :: Patch size : %d x %d is not enough to use the specified region spacing and / or count",
			resx, resy));
	}
	printf("sub regions are of size: %d x %d\n", region_size_x, region_size_y);

	region_bounds_x.resize(params.n_regions_x, Eigen::NoChange);
	region_bounds_y.resize(params.n_regions_y, Eigen::NoChange);
	region_bounds.resize(n_regions, Eigen::NoChange);

	for(int idx = 0; idx < params.n_regions_x; ++idx){
		region_bounds_x(idx, 0) = idx*region_size_x;
		region_bounds_x(idx, 1) = region_bounds_x(idx, 0) + region_size_x - 1;
	}
	region_bounds_x(params.n_regions_x - 1, 1) = resx - 1;

	for(int idy = 0; idy < params.n_regions_y; ++idy){
		region_bounds_y(idy, 0) = idy*region_size_y;
		region_bounds_y(idy, 1) = region_bounds_y(idy, 0) + region_size_y - 1;
	}
	region_bounds_y(params.n_regions_x - 1, 1) = resy - 1;

	region_idx.resize(params.n_regions_y, params.n_regions_x);
	for(int idy = 0; idy < params.n_regions_y; ++idy){
		for(int idx = 0; idx < params.n_regions_x; ++idx){
			region_idx(idy, idx) = idy * params.n_regions_x + idx;
			region_bounds(region_idx(idy, idx), 0) = region_bounds_x(idx, 0);
			region_bounds(region_idx(idy, idx), 1) = region_bounds_x(idx, 1);
			region_bounds(region_idx(idy, idx), 2) = region_bounds_y(idy, 0);
			region_bounds(region_idx(idy, idx), 3) = region_bounds_y(idy, 1);
		}
	}
	printf("Only additive update is currently supported\n");
}

void PGB::apply(double *g, const double *I, const double *p){
	double bias = p[bias_id];
	for(int region_id = 0; region_id < n_regions; ++region_id){
		double gain = (1 + p[region_id]);
		for(int pix_id_x = region_bounds(region_id, 0); pix_id_x < region_bounds(region_id, 1); ++pix_id_x){
			for(int pix_id_y = region_bounds(region_id, 2); pix_id_y < region_bounds(region_id, 3); ++pix_id_y){
				int pix_id = pix_id_y*resx + pix_id_x;
				g[pix_id] = gain * I[pix_id] + bias;
			}
		}
	}
	//showSubRegions(g);
}
void PGB::invert(double *inv_p, const double *p){
	for(int state_id = 0; state_id < state_size; ++state_id){
		inv_p[state_id] = -p[state_id];
	}
}
void PGB::update(double *new_p, const double *old_p, const double *dp){
	for(int state_id = 0; state_id < state_size; ++state_id){
		new_p[state_id] = old_p[state_id] + dp[state_id];
	}
}
void PGB::cmptParamJacobian(double *df_dp, const double *df_dg,
	const double *I, const double *p){
	df_dp[bias_id] = 0;
	for(int region_id = 0; region_id < n_regions; ++region_id){
		df_dp[region_id] = 0;
		for(int pix_id_x = region_bounds(region_id, 0); pix_id_x < region_bounds(region_id, 1); ++pix_id_x){
			for(int pix_id_y = region_bounds(region_id, 2); pix_id_y < region_bounds(region_id, 3); ++pix_id_y){
				int pix_id = pix_id_y*resx + pix_id_x;
				df_dp[region_id] += df_dg[pix_id] * I[pix_id];
				df_dp[bias_id] += df_dg[pix_id];
			}
		}
	}
}
void PGB::cmptPixJacobian(double *df_dI, const double *df_dg,
	const double *I, const double *p){
	// dg_dI = (1 + p[0]) for both additive and compositional formulations
	for(int region_id = 0; region_id < n_regions; ++region_id){
		double gain = (1 + p[region_id]);
		for(int pix_id_x = region_bounds(region_id, 0); pix_id_x < region_bounds(region_id, 1); ++pix_id_x){
			for(int pix_id_y = region_bounds(region_id, 2); pix_id_y < region_bounds(region_id, 3); ++pix_id_y){
				int pix_id = pix_id_y*resx + pix_id_x;
				df_dI[pix_id] = df_dg[pix_id] * gain;
			}
		}
	}
}
void PGB::cmptParamHessian(double *_d2f_dp2, const double *_d2f_dg2,
	const double *I, const double *p){
	Map<MatrixXd> d2f_dp2(_d2f_dp2, state_size, state_size);
	d2f_dp2.fill(0);

	switch(d2f_dg2_type){
	case  PixHessType::Constant:
	{
		//! d2f_dg2 is a constant multiple of identity matrix	
		double d2f_dg2_const = _d2f_dg2 ? *_d2f_dg2 : 1;
		for(int region_id = 0; region_id < n_regions; ++region_id){
			for(int pix_id_x = region_bounds(region_id, 0); pix_id_x < region_bounds(region_id, 1); ++pix_id_x){
				for(int pix_id_y = region_bounds(region_id, 2); pix_id_y < region_bounds(region_id, 3); ++pix_id_y){
					int pix_id = pix_id_y*resx + pix_id_x;
					d2f_dp2(region_id, region_id) += I[pix_id] * I[pix_id] * d2f_dg2_const;
					d2f_dp2(region_id, bias_id) += I[pix_id] * d2f_dg2_const;
				}
			}
			d2f_dp2(bias_id, region_id) = d2f_dp2(region_id, bias_id);
		}
		d2f_dp2(bias_id, bias_id) = n_pix;
		break;
	}
	case  PixHessType::Diagonal:
	{
		break;
	}
	case  PixHessType::General:
	{
		Map<const MatrixXd> d2f_dg2(_d2f_dg2, n_pix, n_pix);
		MatrixXd temp_prod(state_size, n_pix);
		temp_prod.row(bias_id).fill(0);
		for(int region_id = 0; region_id < n_regions; ++region_id){
			temp_prod.row(region_id).fill(0);
			double gain = (1 + p[region_id]);
			for(int pix_id_x = region_bounds(region_id, 0); pix_id_x < region_bounds(region_id, 1); ++pix_id_x){
				for(int pix_id_y = region_bounds(region_id, 2); pix_id_y < region_bounds(region_id, 3); ++pix_id_y){
					int pix_id = pix_id_y*resx + pix_id_x;
					temp_prod(region_id, pix_id) = I[pix_id] * d2f_dg2(pix_id, pix_id);
					d2f_dp2(region_id, region_id) = temp_prod(region_id, pix_id)*I[pix_id];

				}
			}
		}
		temp_prod.row(bias_id) = d2f_dg2.colwise().sum();
		d2f_dp2.col(bias_id) = temp_prod.rowwise().sum();
		break;
	}
	default:
		throw std::invalid_argument(
			cv::format("cmptParamHessian :: Invalid hessian type provided: %d", d2f_dg2_type));
	}
}
void PGB::cmptPixHessian(double *_d2f_dI2, const double *_d2f_dg2,
	const double *I, const double *p){
	Map<VectorXd> d2f_dI2(_d2f_dI2, n_pix);
	d2f_dI2.fill(0);
	switch(d2f_dg2_type){
	case  PixHessType::Constant:
	{

		double d2f_dg2_const = _d2f_dg2 ? *_d2f_dg2 : 1;
		for(int region_id = 0; region_id < n_regions; ++region_id){
			double gain2 = (1 + p[region_id]) *  (1 + p[region_id]);
			for(int pix_id_x = region_bounds(region_id, 0); pix_id_x < region_bounds(region_id, 1); ++pix_id_x){
				for(int pix_id_y = region_bounds(region_id, 2); pix_id_y < region_bounds(region_id, 3); ++pix_id_y){
					int pix_id = pix_id_y*resx + pix_id_x;
					d2f_dI2(pix_id) = gain2*d2f_dg2_const;
				}
			}
		}
		break;
	}
	case  PixHessType::Diagonal:
	{
		Map<const VectorXd> d2f_dg2_diag(_d2f_dg2, n_pix);
		for(int region_id = 0; region_id < n_regions; ++region_id){
			double gain2 = (1 + p[region_id]) *  (1 + p[region_id]);
			for(int pix_id_x = region_bounds(region_id, 0); pix_id_x < region_bounds(region_id, 1); ++pix_id_x){
				for(int pix_id_y = region_bounds(region_id, 2); pix_id_y < region_bounds(region_id, 3); ++pix_id_y){
					int pix_id = pix_id_y*resx + pix_id_x;
					d2f_dI2(pix_id) = gain2 * d2f_dg2_diag(pix_id);
				}
			}
		}
		break;
	}
	case  PixHessType::General:
	{
		MatrixXd dg_dp(n_pix, state_size);
		dg_dp.fill(0);
		for(int region_id = 0; region_id < n_regions; ++region_id){
			double gain2 = (1 + p[region_id]) *  (1 + p[region_id]);
			for(int pix_id_x = region_bounds(region_id, 0); pix_id_x < region_bounds(region_id, 1); ++pix_id_x){
				for(int pix_id_y = region_bounds(region_id, 2); pix_id_y < region_bounds(region_id, 3); ++pix_id_y){
					int pix_id = pix_id_y*resx + pix_id_x;
					dg_dp(pix_id, region_id) = gain2;
				}
			}
		}
		d2f_dI2 = dg_dp.transpose() * Map<const MatrixXd>(_d2f_dg2, n_pix, n_pix) * dg_dp;
		break;
	}
	default:
		throw std::invalid_argument(
			cv::format("cmptPixHessian :: Invalid hessian type provided: %d", d2f_dg2_type));
	}
}
void PGB::cmptCrossHessian(double *_d2f_dp_dI, const double *d2f_dg2,
	const double *I, const double *p){
	if(d2f_dg2){
	} else{
		Map<MatrixXd> d2f_dp_dI(_d2f_dp_dI, state_size, n_pix);
		d2f_dp_dI.fill(0);
		for(int region_id = 0; region_id < n_regions; ++region_id){
			double gain = 1 + p[region_id];
			for(int pix_id_x = region_bounds(region_id, 0); pix_id_x < region_bounds(region_id, 1); ++pix_id_x){
				for(int pix_id_y = region_bounds(region_id, 2); pix_id_y < region_bounds(region_id, 3); ++pix_id_y){
					int pix_id = pix_id_y*resx + pix_id_x;
					d2f_dp_dI(region_id, pix_id) = gain * I[pix_id];
					d2f_dp_dI(bias_id, pix_id) = gain;
				}
			}
		}
	}

}

IlluminationModel::PixHessType PGB::getPixHessType(){
	switch(d2f_dg2_type){
	case  PixHessType::Constant:
		return  PixHessType::Diagonal;
	case  PixHessType::Diagonal:
		return  PixHessType::General;
	case  PixHessType::General:
		return  PixHessType::General;
	default:
		throw std::invalid_argument(
			cv::format("PGB:: d2f_dg2 has nvalid type : %d", d2f_dg2_type));
	}
}


void PGB::showSubRegions(double *patch_data){
	cv::Mat patch_img(resy, resx, CV_8UC1);
	cv::Mat(resy, resx, CV_64FC1, patch_data).convertTo(patch_img, patch_img.type());
	cv::Mat patch_img_rgb(resy, resx, CV_8UC3);
	cv::cvtColor(patch_img, patch_img_rgb, CV_GRAY2BGR);

	cv::Scalar region_cols[8] = {
		cv::Scalar(0, 0, 255),
		cv::Scalar(0, 255, 0),
		cv::Scalar(255, 0, 0),
		cv::Scalar(255, 255, 0),
		cv::Scalar(0, 255, 255),
		cv::Scalar(255, 0, 255),
		cv::Scalar(0, 0, 0),
		cv::Scalar(255, 255, 255)
	};
	int n_cols = sizeof(region_cols) / sizeof(cv::Scalar);

	for(int region_id = 0; region_id < n_regions; ++region_id){
		int min_x = region_bounds(region_id, 0), max_x = region_bounds(region_id, 1);
		int min_y = region_bounds(region_id, 2), max_y = region_bounds(region_id, 3);
		int col_id = region_id % n_cols;

		cv::Point ul(min_x, min_y);
		cv::Point ur(max_x, min_y);
		cv::Point lr(max_x, max_y);
		cv::Point ll(min_x, max_y);
		cv::Point centroid = (ul + ur + lr + ll)*0.25;

		cv::line(patch_img_rgb, ul, ur, region_cols[col_id], 1, CV_AA);
		cv::line(patch_img_rgb, ur, lr, region_cols[col_id], 1, CV_AA);
		cv::line(patch_img_rgb, lr, ll, region_cols[col_id], 1, CV_AA);
		cv::line(patch_img_rgb, ll, ul, region_cols[col_id], 1, CV_AA);

		putText(patch_img_rgb, cv::format("%d", region_id), centroid,
			cv::FONT_HERSHEY_SIMPLEX, 0.50, region_cols[col_id]);
	}
	cv::Mat patch_img_rgb_resized(resy * 4, resx * 4, CV_8UC3);
	cv::resize(patch_img_rgb, patch_img_rgb_resized, cv::Size(patch_img_rgb_resized.cols, patch_img_rgb_resized.rows));
	cv::imshow("PGB Regions", patch_img_rgb_resized);
	char key = cv::waitKey(1);
	if(key == 'n' || key == 'N'){}
}

void PGB::parseSamplerSigma(VectorXd &out_sigma, const VectorXd &in_sigma){
	assert(out_sigma.size() == state_size);
	if(in_sigma.size() == 1){
		out_sigma.fill(in_sigma[0]);
	} else if(in_sigma.size() == 2){
		out_sigma.head(state_size - 1).fill(in_sigma[0]);
		out_sigma[state_size - 1] = in_sigma[1];
	} else{
		if(in_sigma.size() != state_size){
			throw std::invalid_argument(
				cv::format("PGB::parseSamplerSigma :: sampler sigma has invalid size %d which should be %d\n",
				in_sigma.size(), state_size));
		}
		out_sigma = in_sigma;
	}
}
void PGB::parseSamplerMean(VectorXd &out_mean, const VectorXd &in_mean){
	assert(out_mean.size() == state_size);
	if(in_mean.size() == 1){
		out_mean.fill(in_mean[0]);
	} else if(in_mean.size() == 2){
		out_mean.head(state_size-1).fill(in_mean[0]);
		out_mean[state_size - 1] = in_mean[1];
	} else{
		if(in_mean.size() != state_size){
			throw std::invalid_argument(
				cv::format("PGB::parseSamplerMean :: sampler mean has invalid size %d which should be %d\n",
				in_mean.size(), state_size));
		}
		out_mean = in_mean;
	}
}

_MTF_END_NAMESPACE




