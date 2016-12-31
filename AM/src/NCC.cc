#include "mtf/AM/NCC.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/spiUtils.h"
#include "mtf/Utilities/excpUtils.h"

#ifdef ENABLE_TBB
#include "tbb/tbb.h" 
#endif

//! OpenMP scheduler
#ifndef NCC_OMP_SCHD
#define NCC_OMP_SCHD auto
#endif

_MTF_BEGIN_NAMESPACE

//! value constructor
NCCParams::NCCParams(const AMParams *am_params,
bool _fast_hess) :
AMParams(am_params),
fast_hess(_fast_hess){}

//! default/copy constructor
NCCParams::NCCParams(const NCCParams *params) :
AMParams(params),
fast_hess(NCC_FAST_HESS){
	if(params){
		fast_hess = params->fast_hess;
	}
}

NCC::NCC(const ParamType *ncc_params, const int _n_channels) :
AppearanceModel(ncc_params, _n_channels), params(ncc_params){
	name = "ncc";
	printf("\n");
	printf("Using Normalized Cross Correlation AM with...\n");
	printf("fast_hess: %d\n", params.fast_hess);
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);
	printf("grad_eps: %e\n", params.grad_eps);
	printf("hess_eps: %e\n", params.hess_eps);

	pix_norm_mult = 1;
	pix_norm_add = 0;
}

double NCC::getLikelihood() const{
	double d = (1.0 / f) - 1;
	return exp(-params.likelihood_alpha * d*d);
}

void NCC::initializeSimilarity(){
	if(!is_initialized.similarity){
		I0_cntr.resize(patch_size);
		It_cntr.resize(patch_size);
	}
#ifndef DISABLE_SPI
	if(spi_mask){
		I0_mean = utils::getMean(spi_mask, I0, patch_size);
		c = 0;
		for(int patch_id = 0; patch_id < patch_size; patch_id++){
			if(!spi_mask[patch_id]){ continue; }
			I0_cntr[patch_id] = I0[patch_id] - I0_mean;
			c += I0_cntr[patch_id] * I0_cntr[patch_id];
		}
		c = std::sqrt(c);
	} else{
#endif
		I0_mean = I0.mean();
		I0_cntr = (I0.array() - I0_mean);
		c = I0_cntr.norm();
#ifndef DISABLE_SPI
	}
#endif
	if(!is_initialized.similarity){
		f = 1;
		It_mean = I0_mean;
		It_cntr = I0_cntr;
		b = c;

		is_initialized.similarity = true;
	}
}

void NCC::initializeGrad(){
	if(!is_initialized.grad){
		df_dIt.resize(patch_size);
		df_dI0.resize(patch_size);

		df_dI0_ncntr.resize(patch_size);
		df_dIt_ncntr.resize(patch_size);

		I0_cntr_c.resize(patch_size);
		It_cntr_b.resize(patch_size);

		df_dI0.fill(0);
		df_dIt.fill(0);

		df_dI0_ncntr.fill(0);
		df_dIt_ncntr.fill(0);

		df_dI0_ncntr_mean = df_dIt_ncntr_mean = 0;

	}
	I0_cntr_c = I0_cntr.array() / c;

	if(!is_initialized.grad){
		It_cntr_b = I0_cntr_c;
		is_initialized.grad = true;
	}
}

void NCC::updateSimilarity(bool prereq_only){
#ifndef DISABLE_SPI
	if(spi_mask){
		It_mean = utils::getMean(spi_mask, It, patch_size);
		a = b = 0;
		for(int patch_id = 0; patch_id < patch_size; patch_id++){
			if(!spi_mask[patch_id]){ continue; }
			It_cntr[patch_id] = It[patch_id] - It_mean;
			a += I0_cntr[patch_id] * It_cntr[patch_id];
			b += It_cntr[patch_id] * It_cntr[patch_id];
		}
		b = std::sqrt(b);
	} else{
#endif
		It_mean = It.mean();
		a = b = 0;
		for(int patch_id = 0; patch_id < patch_size; ++patch_id){
			It_cntr[patch_id] = It[patch_id] - It_mean;
			a += I0_cntr[patch_id] * It_cntr[patch_id];
			b += It_cntr[patch_id] * It_cntr[patch_id];
		}
		b = std::sqrt(b);

		//It_cntr = (It.array() - It_mean);
		//a = (I0_cntr.array() * It_cntr.array()).sum();
		//b = It_cntr.norm();

#ifndef DISABLE_SPI
	}
#endif
	bc = b*c;
	b2c = bc*b;
	f = a / bc;
}

void NCC::updateInitGrad(){
#ifndef DISABLE_SPI
	if(spi_mask){
		df_dI0_ncntr_mean = 0;
		int valid_patch_size = 0;
		for(int patch_id = 0; patch_id < patch_size; patch_id++){
			if(!spi_mask[patch_id]){ continue; }
			It_cntr_b(patch_id) = It_cntr(patch_id) / b;
			df_dI0_ncntr(patch_id) = (It_cntr_b(patch_id) - f*I0_cntr_c(patch_id)) / c;
			df_dI0_ncntr_mean += df_dI0_ncntr(patch_id);
			++valid_patch_size;
		}
		df_dI0_ncntr_mean /= valid_patch_size;
	} else{
#endif
		df_dI0_ncntr_mean = 0;
		for(int patch_id = 0; patch_id < patch_size; patch_id++){
			It_cntr_b(patch_id) = It_cntr(patch_id) / b;
			df_dI0_ncntr(patch_id) = (It_cntr_b(patch_id) - f*I0_cntr_c(patch_id)) / c;
			df_dI0_ncntr_mean += df_dI0_ncntr(patch_id);
		}
		df_dI0_ncntr_mean /= patch_size;
		//df_dI0_ncntr_mean = df_dI0_ncntr.mean();	
#ifndef DISABLE_SPI
	}
#endif
	df_dI0 = df_dI0_ncntr.array() - df_dI0_ncntr_mean;
}

void NCC::updateCurrGrad(){
#ifndef DISABLE_SPI
	if(spi_mask){
		df_dIt_ncntr_mean = 0;
		int valid_patch_size = 0;
		for(int patch_id = 0; patch_id < patch_size; patch_id++){
			if(!spi_mask[patch_id]){ continue; }
			It_cntr_b(patch_id) = It_cntr(patch_id) / b;
			df_dIt_ncntr(patch_id) = (I0_cntr_c(patch_id) - f*It_cntr_b(patch_id)) / b;
			df_dIt_ncntr_mean += df_dIt_ncntr(patch_id);
			++valid_patch_size;
		}
		df_dIt_ncntr_mean /= valid_patch_size;
	} else{
#endif
		df_dIt_ncntr_mean = 0;
		for(int patch_id = 0; patch_id < patch_size; patch_id++){
			It_cntr_b(patch_id) = It_cntr(patch_id) / b;
			df_dIt_ncntr(patch_id) = (I0_cntr_c(patch_id) - f*It_cntr_b(patch_id)) / b;
			df_dIt_ncntr_mean += df_dIt_ncntr(patch_id);
		}
		df_dIt_ncntr_mean /= patch_size;
		//df_dIt_ncntr_mean = df_dIt_ncntr.mean();
#ifndef DISABLE_SPI
	}
#endif
	df_dIt = df_dIt_ncntr.array() - df_dIt_ncntr_mean;

}

void NCC::cmptInitJacobian(RowVectorXd &df_dp,
	const MatrixXd &dI0_dp){
	assert(df_dp.size() == dI0_dp.cols());
	assert(dI0_dp.rows() == patch_size);
#ifndef DISABLE_SPI
	if(spi_mask){
		utils::getProd(df_dp, spi_mask, df_dI0, dI0_dp,
			n_pix, n_channels);
	} else{
#endif
		df_dp.noalias() = df_dI0 * dI0_dp;
#ifndef DISABLE_SPI
	}
#endif
}

void NCC::cmptCurrJacobian(RowVectorXd &df_dp,
	const MatrixXd &dIt_dp){
	assert(df_dp.size() == state_size + dIt_dp.cols());
	assert(dIt_dp.rows() == patch_size);
#ifndef DISABLE_SPI
	if(spi_mask){
		utils::getProd(df_dp, spi_mask, df_dIt, dIt_dp,
			n_pix, n_channels);
	} else{
#endif
		df_dp.noalias() = df_dIt * dIt_dp;
#ifndef DISABLE_SPI
	}
#endif
}

void NCC::cmptDifferenceOfJacobians(RowVectorXd &df_dp_diff,
	const MatrixXd &dI0_dp, const MatrixXd &dIt_dp){
#ifndef DISABLE_SPI
	if(spi_mask){
		utils::getDiffOfProd(df_dp_diff, spi_mask, df_dIt, dIt_dp,
			df_dI0, dI0_dp, n_pix, n_channels);
	} else{
#endif
		df_dp_diff.noalias() = (df_dIt * dIt_dp) - (df_dI0 * dI0_dp);
#ifndef DISABLE_SPI
	}
#endif
}

void NCC::cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dp){
	int ssm_state_size = d2f_dp2.rows();
	assert(d2f_dp2.cols() == ssm_state_size);
#ifndef DISABLE_SPI
	if(spi_mask){
		utils::FunctonNotImplemented("NCC::cmptInitHessian: SPI support not implemented yet");
	} else{
#endif
		MatrixXd dI0_dpssm_cntr = (dI0_dp.rowwise() - dI0_dp.colwise().mean()).array() / b;
		d2f_dp2 =
			-f*dI0_dpssm_cntr.transpose()*dI0_dpssm_cntr
			-
			(dI0_dpssm_cntr.transpose()*It_cntr_b)*(I0_cntr_c.transpose()*dI0_dpssm_cntr)
			-
			(dI0_dpssm_cntr.transpose()*I0_cntr_c)*(It_cntr_b.transpose()*dI0_dpssm_cntr)
			+
			3 * (dI0_dpssm_cntr.transpose()*I0_cntr_c)*(I0_cntr_c.transpose()*dI0_dpssm_cntr);
#ifndef DISABLE_SPI
	}
#endif
}
void NCC::cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp){
	int ssm_state_size = d2f_dp2.rows();
	assert(d2f_dp2.cols() == ssm_state_size);
#ifndef DISABLE_SPI
	if(spi_mask){
		utils::FunctonNotImplemented("NCC::cmptCurrHessian: SPI support not implemented yet");
	} else{
#endif
		//curr_hessian.fill(0);
		//double b2 = b*b;
		//double additive_factor = similarity / b2;
		//for(int j = 0; j < n_pix; j++){
		//	//double hess_mean = 0;
		//	hess = (3 * curr_pix_vals_cntr_b(j)*curr_pix_vals_cntr_b -
		//		curr_pix_vals_cntr_b(j)*init_pix_vals_cntr_c -
		//		init_pix_vals_cntr_c(j)*curr_pix_vals_cntr_b).array() / b2;
		//	hess(j) += additive_factor;
		//	hess.array() -= hess.mean();
		//	curr_hessian += curr_pix_jacobian.transpose() * hess * curr_pix_jacobian.row(j);
		//}
		MatrixXd dIt_dpssm_cntr = (dIt_dp.rowwise() - dIt_dp.colwise().mean()).array() / b;
		d2f_dp2 =
			-f*dIt_dpssm_cntr.transpose()*dIt_dpssm_cntr
			-
			(dIt_dpssm_cntr.transpose()*It_cntr_b)*(I0_cntr_c.transpose()*dIt_dpssm_cntr)
			-
			(dIt_dpssm_cntr.transpose()*I0_cntr_c)*(It_cntr_b.transpose()*dIt_dpssm_cntr)
			+
			3 * (dIt_dpssm_cntr.transpose()*It_cntr_b)*(It_cntr_b.transpose()*dIt_dpssm_cntr);
		//utils::printMatrix(curr_hessian, "curr_hessian");
#ifndef DISABLE_SPI
	}
#endif
}
void NCC::cmptSelfHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp){
	assert(d2f_dp2.cols() == d2f_dp2.rows());	
#ifndef DISABLE_SPI
	if(spi_mask){
		RowVectorXd dIt_dp_mean(dIt_dp.cols());
		utils::getMean(dIt_dp_mean, spi_mask, dIt_dp, patch_size);
		MatrixXd dIt_dp_cntr = (dIt_dp.rowwise() - dIt_dp_mean).array() / b;
		if(params.fast_hess){
			d2f_dp2.setZero();
			for(int patch_id = 0; patch_id < patch_size; patch_id++){
				if(!spi_mask[patch_id]){ continue; }
				d2f_dp2 += -dIt_dp_cntr.row(patch_id).transpose()*dIt_dp_cntr.row(patch_id)
					+
					dIt_dp_cntr.row(patch_id).transpose()*
					(It_cntr_b[patch_id] * It_cntr_b[patch_id])*
					dIt_dp_cntr.row(patch_id);
			}
		} else{
			d2f_dp2.setZero();
			for(int patch_id = 0; patch_id < patch_size; patch_id++){
				if(!spi_mask[patch_id]){ continue; }
				d2f_dp2 += -dIt_dp_cntr.row(patch_id).transpose()*dIt_dp_cntr.row(patch_id)
					+
					(dIt_dp_cntr.row(patch_id).transpose()*It_cntr_b(patch_id))
					*(It_cntr_b[patch_id] * dIt_dp_cntr.row(patch_id));
			}
		}
	} else{
#endif	
		MatrixXd dIt_dp_cntr = (dIt_dp.rowwise() - dIt_dp.colwise().mean()).array() / b;
		//printf("curr_pix_jacobian size: %ld, %ld\t", curr_pix_jacobian.rows(), curr_pix_jacobian.cols());
		//printf("curr_pix_jacobian_cntr size: %ld, %ld\t", curr_pix_jacobian_cntr.rows(), curr_pix_jacobian_cntr.cols());
		//printf("curr_pix_vals_cntr_b size: %ld, %ld\n", curr_pix_vals_cntr_b.rows(), curr_pix_vals_cntr_b.cols());
		//utils::printMatrixToFile(curr_pix_jacobian, "curr_pix_jacobian", "log/ncc_log.txt", "%15.9f", "w");
		//utils::printMatrixToFile(curr_pix_jacobian_cntr, "curr_pix_jacobian_cntr", "log/ncc_log.txt", "%15.9f", "a");
		//utils::printMatrixToFile(curr_pix_vals_cntr_b, "curr_pix_vals_cntr_b", "log/ncc_log.txt", "%15.9f", "a");
		if(params.fast_hess){
			d2f_dp2 =
				-dIt_dp_cntr.transpose()*dIt_dp_cntr
				+
				dIt_dp_cntr.transpose()*(
				It_cntr_b.transpose()*It_cntr_b
				)*dIt_dp_cntr;
		} else{
			d2f_dp2 =
				-dIt_dp_cntr.transpose()*dIt_dp_cntr
				+
				(dIt_dp_cntr.transpose()*It_cntr_b)*(It_cntr_b.transpose()*dIt_dp_cntr);
		}
#ifndef DISABLE_SPI
	}
#endif
}

void NCC::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
	const MatrixXd &init_pix_hessian){
	int ssm_state_size = init_hessian.rows();
	assert(init_pix_hessian.rows() == ssm_state_size * ssm_state_size);
	cmptInitHessian(init_hessian, init_pix_jacobian);
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		spi_pt_ckeck(pix_id);
		init_hessian += Map<const MatrixXd>(init_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size) * df_dI0(pix_id);;
	}
}
void NCC::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int ssm_state_size = curr_hessian.rows();
	assert(curr_pix_hessian.rows() == ssm_state_size * ssm_state_size);
	cmptCurrHessian(curr_hessian, curr_pix_jacobian);
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		spi_pt_ckeck(pix_id);
		curr_hessian += Map<const MatrixXd>(curr_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size) * df_dIt(pix_id);;
	}
}

void NCC::estimateOpticalFlow(std::vector<cv::Point2f> &curr_pts, const cv::Mat &prev_img,
	const std::vector<cv::Point2f> &prev_pts, cv::Size search_window, int n_pts,
	int max_iters, double term_eps, bool const_grad) const{
	assert(curr_pts.size() == n_pts && prev_pts.size() == n_pts);

	double half_width = search_window.width / 2.0, half_height = search_window.height / 2.0;
	int srch_size = search_window.width*search_window.height;
	double grad_mult_factor = 1.0 / (2 * grad_eps);

#ifdef ENABLE_TBB
	parallel_for(tbb::blocked_range<size_t>(0, n_pts),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t pt_id = r.begin(); pt_id != r.end(); ++pt_id){
#else
#pragma omp parallel for schedule(NCC_OMP_SCHD)
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
#endif	
		VectorXd _I0(srch_size),_I0_cntr(srch_size);		
		PixGradT _dIt_dx(srch_size, 2);

		VectorXd x_vals = VectorXd::LinSpaced(search_window.width,
			prev_pts[pt_id].x - half_width, prev_pts[pt_id].x + half_width);
		VectorXd y_vals = VectorXd::LinSpaced(search_window.height,
			prev_pts[pt_id].y - half_height, prev_pts[pt_id].y + half_height);

		double _I0_mean = 0;
		RowVector2d _dIt_dx_mean(0, 0);
		for(int srch_id = 0; srch_id < srch_size; ++srch_id){
			double x = x_vals(srch_id % search_window.width), y = y_vals(srch_id / search_window.width);

			_I0(srch_id) = utils::getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(prev_img,
				x, y, img_height, img_width);
			_I0_mean += _I0(srch_id);

			if(!const_grad){ continue; }

			double pix_val_inc = utils::getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(prev_img,
				x + grad_eps, y, img_height, img_width);
			double  pix_val_dec = utils::getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(prev_img,
				x - grad_eps, y, img_height, img_width);
			_dIt_dx(srch_id, 0) = (pix_val_inc - pix_val_dec)*grad_mult_factor;

			pix_val_inc = utils::getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(prev_img,
				x, y + grad_eps, img_height, img_width);
			pix_val_dec = utils::getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(prev_img,
				x, y - grad_eps, img_height, img_width);

			_dIt_dx(srch_id, 1) = (pix_val_inc - pix_val_dec)*grad_mult_factor;
			_dIt_dx_mean += _dIt_dx.row(srch_id);
		}
		_I0_mean /= srch_size;
		if(const_grad){ _dIt_dx_mean.array() /= srch_size; }

		double _c = 0;
		for(int srch_id = 0; srch_id < srch_size; ++srch_id){
			_I0_cntr[srch_id] = _I0[srch_id] - _I0_mean;
			_c += _I0_cntr[srch_id] * _I0_cntr[srch_id];
		}
		_c = std::sqrt(_c);
		VectorXd _I0_cntr_c = _I0_cntr.array() / _c;

		curr_pts[pt_id] = prev_pts[pt_id];
		for(int iter_id = 0; iter_id < max_iters; ++iter_id){			
			if(iter_id > 0){
				x_vals = VectorXd::LinSpaced(search_window.width,
					curr_pts[pt_id].x - half_width, curr_pts[pt_id].x + half_width);
				y_vals = VectorXd::LinSpaced(search_window.height,
					curr_pts[pt_id].y - half_height, curr_pts[pt_id].y + half_height);
			}
			double _It_mean = 0;
			VectorXd _It(srch_size), _It_cntr(srch_size);
			if(!const_grad){ _dIt_dx_mean.setZero(); }
			for(int srch_id = 0; srch_id < srch_size; ++srch_id){
				double x = x_vals(srch_id % search_window.width), y = y_vals(srch_id / search_window.width);
				_It(srch_id) = utils::getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(curr_img,
					x, y, img_height, img_width);
				_It_mean += _It(srch_id);

				if(const_grad){ continue; }

				double pix_val_inc = utils::getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(curr_img,
					x + grad_eps, y, img_height, img_width);
				double  pix_val_dec = utils::getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(curr_img,
					x - grad_eps, y, img_height, img_width);
				_dIt_dx(srch_id, 0) = (pix_val_inc - pix_val_dec)*grad_mult_factor;

				pix_val_inc = utils::getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(curr_img,
					x, y + grad_eps, img_height, img_width);
				pix_val_dec = utils::getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(curr_img,
					x, y - grad_eps, img_height, img_width);

				_dIt_dx(srch_id, 1) = (pix_val_inc - pix_val_dec)*grad_mult_factor;
				_dIt_dx_mean += _dIt_dx.row(srch_id);
			}
			_It_mean /= srch_size;
			if(!const_grad){ _dIt_dx_mean.array() /= srch_size; }

			double _a = 0, _b = 0;
			for(int srch_id = 0; srch_id < srch_size; ++srch_id){
				_It_cntr[srch_id] = _It[srch_id] - _It_mean;

				_a += _I0_cntr[srch_id] * _It_cntr[srch_id];
				_b += _It_cntr[srch_id] * _It_cntr[srch_id];
			}
			_b = std::sqrt(_b);

			double _f = _a / (_b*_c);

			double _df_dIt_ncntr_mean = 0;
			VectorXd _df_dIt_ncntr(srch_size), _It_cntr_b(srch_size);
			for(int srch_id = 0; srch_id < srch_size; ++srch_id){
				_It_cntr_b(srch_id) = _It_cntr(srch_id) / _b;
				_df_dIt_ncntr(srch_id) = (_I0_cntr_c(srch_id) - _f*_It_cntr_b(srch_id)) / _b;
				_df_dIt_ncntr_mean += _df_dIt_ncntr(srch_id);
			}
			_df_dIt_ncntr_mean /= srch_size;

			RowVector2d _df_dx(0, 0);
			PixGradT _dIt_dx_cntr(srch_size, 2);
			for(int srch_id = 0; srch_id < srch_size; ++srch_id){
				_df_dx += (_df_dIt_ncntr(srch_id) - _df_dIt_ncntr_mean)*_dIt_dx.row(srch_id);
				_dIt_dx_cntr.row(srch_id) = (_dIt_dx.row(srch_id) - _dIt_dx_mean).array() / _b;
			}
			Matrix2d _d2f_dx2 =
				-_dIt_dx_cntr.transpose()*_dIt_dx_cntr
				+
				(_dIt_dx_cntr.transpose()*_It_cntr_b)*(_It_cntr_b.transpose()*_dIt_dx_cntr);

			//double _d2f_dx2_det = _d2f_dx2(0, 0)*_d2f_dx2(1, 1) - _d2f_dx2(0, 1)*_d2f_dx2(1, 0);
			//if(_d2f_dx2_det == 0.0){ break; }
			//double opt_flow_x = (_d2f_dx2(1, 1)*_df_dx(0) - _d2f_dx2(0, 1)*_df_dx(1)) / _d2f_dx2_det;
			//double opt_flow_y = (_d2f_dx2(0, 0)*_df_dx(1) - _d2f_dx2(1, 0)*_df_dx(0)) / _d2f_dx2_det;
			//curr_pts[pt_id].x += opt_flow_x;
			//curr_pts[pt_id].y += opt_flow_y;
			//if(opt_flow_x*opt_flow_x + opt_flow_y*opt_flow_y < term_eps){ break; }

			Vector2d opt_flow = -_d2f_dx2.colPivHouseholderQr().solve(_df_dx.transpose());
			curr_pts[pt_id].x += opt_flow[0];
			curr_pts[pt_id].y += opt_flow[1];
			if(opt_flow.squaredNorm() < term_eps){ break; }
		}
	}
#ifdef ENABLE_TBB
	});
#endif	
}

/*Support for FLANN library*/

void NCC::updateDistFeat(double* feat_addr){
	It_mean = It.mean();
	double curr_pix_std = sqrt((It.array() - It_mean).matrix().squaredNorm());
	for(size_t pix = 0; pix < patch_size; pix++) {
		*feat_addr = (It(pix) - It_mean) / curr_pix_std;
		feat_addr++;
	}
}

double NCC::operator()(const double* a, const double* b,
	size_t size, double worst_dist) const {
	assert(size == patch_size);

	double num = 0;
	const double* last = a + size;
	const double* lastgroup = last - 3;

	/* Process 4 items with each loop for efficiency. */
	while(a < lastgroup) {

		num += a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
		a += 4;
		b += 4;
	}
	/* Process last 0-3 pixels.  Not needed for standard vector lengths. */
	while(a < last) {

		num += *a * *b;
		a++;
		b++;
	}
	return -num;
}

_MTF_END_NAMESPACE

