#include "CCRE.h"
#include "../Utilities/histUtils.h"
#include "tbb/tbb.h" 

_MTF_BEGIN_NAMESPACE

void CCRE::update(bool prereq_only){

	// compute both the histogram and its differential simultaneously
	// to take advantage of the many common computations involved

	curr_cum_hist.fill(hist_pre_seed);
	curr_cum_joint_hist.fill(params.pre_seed);
	curr_cum_hist_grad.setZero();
	curr_cum_joint_hist_grad.setZero();

	static tbb::affinity_partitioner ap;
	parallel_for(tbb::blocked_range<size_t>(0, params.n_blocks),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				_curr_bspl_ids.row(pix_id) = _std_bspl_ids.row(static_cast<int>(curr_pix_vals(pix_id)));
				int curr_cum_hist_id = 0;
				while(curr_cum_hist_id < _curr_bspl_ids(pix_id, 0)){// curr_cum_hist_mat is 1 and curr_cum_hist_grad is zero but the latter has already been zeroed
					curr_cum_hist(curr_cum_hist_id) += curr_cum_hist_mat(curr_cum_hist_id, pix_id) = 1;
					for(int hist_id = _init_bspl_ids(pix_id, 0); hist_id <= _init_bspl_ids(pix_id, 1); hist_id++) {
						curr_cum_joint_hist(curr_cum_hist_id, hist_id) += init_hist_mat(hist_id, pix_id);
					}
					++curr_cum_hist_id;
				}
				double curr_diff = curr_cum_hist_id - curr_pix_vals(pix_id);
				while(curr_cum_hist_id <= _curr_bspl_ids(pix_id, 1)){
					// since the ids of all bins affected by a pix_idel are sequential, repeated computation
					// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
					utils::cumBSpl3WithGrad(curr_cum_hist_mat(curr_cum_hist_id, pix_id), curr_cum_hist_grad(curr_cum_hist_id, pix_id), curr_diff);
					curr_cum_hist_grad(curr_cum_hist_id, pix_id) = -hist_norm_mult*curr_cum_hist_grad(curr_cum_hist_id, pix_id);
					curr_cum_hist(curr_cum_hist_id) += curr_cum_hist_mat(curr_cum_hist_id, pix_id);
					for(int hist_id = _init_bspl_ids(pix_id, 0); hist_id <= _init_bspl_ids(pix_id, 1); hist_id++) {
						curr_cum_joint_hist(curr_cum_hist_id, hist_id) += curr_cum_hist_mat(curr_cum_hist_id, pix_id) * init_hist_mat(hist_id, pix_id);
						curr_cum_joint_hist_grad(_linear_idx(curr_cum_hist_id, hist_id), pix_id) =
							curr_cum_hist_grad(curr_cum_hist_id, pix_id) * init_hist_mat(hist_id, pix_id);
					}
					++curr_diff;
					++curr_cum_hist_id;
				}
			}
		}
	}, ap);
	curr_cum_hist *= hist_norm_mult;
	curr_cum_joint_hist *= hist_norm_mult;

	curr_cum_hist_log = curr_cum_hist.array().log();
	curr_cum_joint_hist_log = curr_cum_joint_hist.array().log();

	for(int id1 = 0; id1 < params.n_bins; id1++){
		for(int id2 = 0; id2 < params.n_bins; id2++){
			curr_ccre_log_term(id1, id2) = curr_cum_joint_hist_log(id1, id2) - curr_cum_hist_log(id1) - init_hist_log(id2);
		}
	}
	if(prereq_only){ return; }

	similarity = (curr_cum_joint_hist.array()*curr_ccre_log_term.array()).sum();
}
void CCRE::updateInitGrad(){
	if(params.symmetrical_grad){
		updateSymInitSimGrad();
		return;
	}
	// compute differential of the current joint histogram w.r.t. initial pixel values simultaneously with init_grad;
	// this does not need to be normalized since init_hist_grad has already been normalized
	_eig_set_zero(init_cum_joint_hist_grad, double);


	parallel_for(tbb::blocked_range<size_t>(0, params.n_blocks),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				init_grad(pix_id) = 0;
				for(int hist_id = _init_bspl_ids(pix_id, 0); hist_id <= _init_bspl_ids(pix_id, 1); hist_id++) {
					int cum_hist_id = 0;
					while(cum_hist_id < _curr_bspl_ids(pix_id, 0)){
						init_cum_joint_hist_grad(_linear_idx(cum_hist_id, hist_id), pix_id) = init_hist_grad(hist_id, pix_id);
						++cum_hist_id;
					}
					while(cum_hist_id <= _curr_bspl_ids(pix_id, 1)){
						int idx = _linear_idx(cum_hist_id, hist_id);
						init_cum_joint_hist_grad(idx, pix_id) =
							curr_cum_hist_mat(cum_hist_id, pix_id) * init_hist_grad(hist_id, pix_id);
						init_grad_2(idx, pix_id) = curr_cum_joint_hist(cum_hist_id, hist_id) * init_hist_grad_ratio(hist_id, pix_id);
						init_grad(pix_id) += init_cum_joint_hist_grad(idx, pix_id) * (1 + curr_ccre_log_term(cum_hist_id, hist_id))
							- init_grad_2(idx, pix_id);
						++cum_hist_id;
					}
				}
			}
		}
	});
}
void  CCRE::cmptCurrHessian(MatrixXd &hessian, const MatrixXd &curr_pix_jacobian){
	int ssm_state_size = curr_pix_jacobian.cols();
	assert(hessian.rows() == ssm_state_size && hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == n_pix);

	// compute curr_cum_hist_hess simultaneously with the error norm hessian
	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	hessian.setZero();
	curr_cum_hist_hess.setZero();
	joint_hist_jacobian.setZero();

	parallel_for(tbb::blocked_range<size_t>(0, params.n_blocks),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				double curr_diff = _curr_bspl_ids(pix_id, 0) - curr_pix_vals(pix_id);
				int curr_start = _curr_bspl_ids(pix_id, 0), curr_end = _curr_bspl_ids(pix_id, 1);
				int init_start = _init_bspl_ids(pix_id, 0), init_end = _init_bspl_ids(pix_id, 1);
				double hist_hess_term = 0;
				for(int curr_id = curr_start; curr_id <= curr_end; curr_id++) {
					double inner_term = 0;
					for(int init_id = init_start; init_id <= init_end; init_id++) {
						int idx = _linear_idx(curr_id, init_id);
						joint_hist_jacobian.row(idx) += curr_cum_joint_hist_grad(idx, pix_id)*curr_pix_jacobian.row(pix_id);
						inner_term += init_hist_mat(init_id, pix_id) * curr_ccre_log_term(curr_id, init_id);
					}
					curr_cum_hist_hess(curr_id, pix_id) = hist_norm_mult*utils::cumBSpl3Hess(curr_diff++);
					hist_hess_term += curr_cum_hist_hess(curr_id, pix_id)*inner_term;
				}
				hessian += hist_hess_term * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id);
			}
		}
	}
	);
	for(int r = 0; r < params.n_bins; r++){
		for(int t = 0; t < params.n_bins; t++){
			int idx = _linear_idx(r, t);
			double hist_factor = (1.0 / curr_cum_joint_hist(r, t)) - (1.0 / curr_cum_hist(r));
			hessian += joint_hist_jacobian.row(idx).transpose() * joint_hist_jacobian.row(idx) * hist_factor;
		}
	}
}
void  CCRE::cmptInitHessian(MatrixXd &hessian, const MatrixXd &init_pix_jacobian){
	if(params.symmetrical_grad){
		cmptSymInitSimHessian(hessian, init_pix_jacobian);
		return;
	}
	assert(hessian.rows() == hessian.cols() && hessian.cols() == init_pix_jacobian.cols());
	assert(init_pix_jacobian.rows() == n_pix);
	hessian.setZero();
	MatrixXd init_cum_joint_hist_jac = init_cum_joint_hist_grad*init_pix_jacobian;
	MatrixXd init_hist_grad_ratio_jac = init_hist_grad_ratio*init_pix_jacobian;

	for(int t = 0; t < params.n_bins; t++) {
		curr_cum_joint_hist_sum(t) = 0;
		for(int r = 0; r < params.n_bins; r++) {
			double joint_hist = curr_cum_joint_hist(r, t);
			int idx = _linear_idx(r, t);

			hessian += init_cum_joint_hist_jac.row(idx).transpose()*
				(init_cum_joint_hist_jac.row(idx) / joint_hist
				-
				2 * init_hist_grad_ratio_jac.row(t));
			curr_cum_joint_hist_sum(t) += joint_hist;
		}
		hessian += curr_cum_joint_hist_sum(t)*init_hist_grad_ratio_jac.row(t).transpose()*init_hist_grad_ratio_jac.row(t);

	}
	parallel_for(tbb::blocked_range<size_t>(0, params.n_blocks),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				double scalar_term = 0;
				for(int t = _init_bspl_ids(pix_id, 0); t <= _init_bspl_ids(pix_id, 1); t++) {
					for(int r = 0; r < params.n_bins; r++) {
						double joint_hist_hess = init_hist_hess(t, pix_id) * curr_cum_hist_mat(r, pix_id);
						scalar_term += joint_hist_hess*(curr_ccre_log_term(r, t) + 1);
					}
					scalar_term -= init_hist_hess_ratio(t, pix_id)*curr_cum_joint_hist_sum(t);
				}
				hessian += scalar_term* init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id);
			}
		}
	});
}
void CCRE::cmptCumSelfHist(){
	// compute curr_hist, curr_hist_mat, curr_cum_hist_hess and curr_cum_joint_hist in a single pass over the pixels;
	curr_hist.fill(hist_pre_seed);
	self_cum_joint_hist.fill(params.pre_seed);
	_eig_set_zero(curr_hist_mat, double);
	_eig_set_zero(curr_cum_hist_hess, double);

	parallel_for(tbb::blocked_range<size_t>(0, params.n_blocks),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				double curr_diff = _curr_bspl_ids(pix_id, 0) - curr_pix_vals(pix_id);
				for(int hist_id = _curr_bspl_ids(pix_id, 0); hist_id <= _curr_bspl_ids(pix_id, 1); ++hist_id) {
					curr_hist_mat(hist_id, pix_id) = utils::bSpl3(curr_diff);
					curr_cum_hist_hess(hist_id, pix_id) = hist_norm_mult*utils::cumBSpl3Hess(curr_diff);
					++curr_diff;
					curr_hist(hist_id) += curr_hist_mat(hist_id, pix_id);

					int cum_hist_id = 0;
					while(cum_hist_id < _curr_bspl_ids(pix_id, 0)){
						self_cum_joint_hist(cum_hist_id, hist_id) += curr_hist_mat(hist_id, pix_id);
						++cum_hist_id;
					}
					while(cum_hist_id <= _curr_bspl_ids(pix_id, 1)){
						self_cum_joint_hist(cum_hist_id, hist_id) += curr_cum_hist_mat(cum_hist_id, pix_id) * curr_hist_mat(hist_id, pix_id);
						++cum_hist_id;
					}
				}
			}
		}
	});

	curr_hist *= hist_norm_mult;
	self_cum_joint_hist *= hist_norm_mult;

	self_cum_joint_hist_log = self_cum_joint_hist.array().log();
	curr_hist_log = curr_hist.array().log();

	for(int i = 0; i < params.n_bins; i++){
		for(int j = 0; j < params.n_bins; j++){
			self_ccre_log_term(i, j) = self_cum_joint_hist_log(i, j) - curr_cum_hist_log(i) - curr_hist_log(j);
		}
	}
}
void CCRE::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian){
	int ssm_state_size = curr_pix_jacobian.cols();

	assert(self_hessian.rows() == ssm_state_size && self_hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == n_pix);

	cmptCumSelfHist();

	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);

	self_hessian.setZero();
	joint_hist_jacobian.setZero();

	parallel_for(tbb::blocked_range<size_t>(0, params.n_blocks),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				double hist_hess_term = 0;
				for(int r = _curr_bspl_ids(pix_id, 0); r <= _curr_bspl_ids(pix_id, 1); ++r) {
					double inner_term = 0;
					for(int t = _curr_bspl_ids(pix_id, 0); t <= _curr_bspl_ids(pix_id, 1); ++t) {
						int idx = _linear_idx(r, t);
						joint_hist_jacobian.row(idx) += curr_cum_hist_grad(r, pix_id) * curr_hist_mat(t, pix_id)*curr_pix_jacobian.row(pix_id);
						inner_term += curr_hist_mat(t, pix_id) * self_ccre_log_term(r, t);
					}
					hist_hess_term += curr_cum_hist_hess(r, pix_id)*inner_term;
				}
				self_hessian += hist_hess_term * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id);
			}
		}
	});

	for(int r = 0; r < params.n_bins; ++r){
		for(int t = 0; t < params.n_bins; ++t){
			int idx = _linear_idx(r, t);
			double hist_factor = (1.0 / self_cum_joint_hist(r, t)) - (1.0 / curr_cum_hist(r));
			self_hessian += joint_hist_jacobian.row(idx).transpose() * joint_hist_jacobian.row(idx) * hist_factor;
		}
	}
}
void CCRE::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int ssm_state_size = curr_pix_jacobian.cols();

	assert(self_hessian.rows() == ssm_state_size && self_hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == n_pix);
	assert(curr_pix_hessian.rows() == ssm_state_size*ssm_state_size);

	cmptCumSelfHist();

	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	self_hessian.setZero();
	joint_hist_jacobian.setZero();


	parallel_for(tbb::blocked_range<size_t>(0, params.n_blocks),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				double hist_hess_term = 0, hist_grad_term = 0;
				for(int r = _curr_bspl_ids(pix_id, 0); r <= _curr_bspl_ids(pix_id, 1); r++) {
					double inner_term = 0;
					for(int t = _curr_bspl_ids(pix_id, 0); t <= _curr_bspl_ids(pix_id, 1); t++) {
						int idx = _linear_idx(r, t);
						joint_hist_jacobian.row(idx) += curr_cum_hist_grad(r, pix_id) * curr_hist_mat(t, pix_id)*curr_pix_jacobian.row(pix_id);
						inner_term += curr_hist_mat(t, pix_id) * self_ccre_log_term(r, t);
					}
					hist_hess_term += curr_cum_hist_hess(r, pix_id)*inner_term;
					hist_grad_term += curr_cum_hist_grad(r, pix_id)*inner_term;
				}
				self_hessian += hist_hess_term * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id)
					+ hist_grad_term * MatrixXdM((double*)curr_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size);
			}
		}
	});

	for(int r = 0; r < params.n_bins; r++){
		for(int t = 0; t < params.n_bins; t++){
			int idx = _linear_idx(r, t);
			double hist_factor = (1.0 / self_cum_joint_hist(r, t)) - (1.0 / curr_cum_hist(r));
			self_hessian += joint_hist_jacobian.row(idx).transpose() * joint_hist_jacobian.row(idx) * hist_factor;
		}
	}
}
// functions for symmetrical init_grad
void CCRE::updateSymInitSimGrad(){

	curr_hist.fill(hist_pre_seed);
	init_cum_joint_hist.fill(params.pre_seed);

	_eig_set_zero(curr_hist_mat, double);
	_eig_set_zero(init_cum_joint_hist_grad, double);

	parallel_for(tbb::blocked_range<size_t>(0, params.n_blocks),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				double curr_diff = _curr_bspl_ids(pix_id, 0) - curr_pix_vals(pix_id);
				for(int hist_id = _curr_bspl_ids(pix_id, 0); hist_id <= _curr_bspl_ids(pix_id, 1); ++hist_id) {
					// since the ids of all bins affected by a pixel are sequential, repeated computation
					// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
					curr_hist_mat(hist_id, pix_id) = utils::bSpl3(curr_diff++);
					curr_hist(hist_id) += curr_hist_mat(hist_id, pix_id);
					int init_cum_hist_id = 0;
					while(init_cum_hist_id < _init_bspl_ids(pix_id, 0)){
						init_cum_joint_hist(init_cum_hist_id, hist_id) += curr_hist_mat(hist_id, pix_id);
						++init_cum_hist_id;
					}
					while(init_cum_hist_id <= _init_bspl_ids(pix_id, 1)){
						init_cum_joint_hist(init_cum_hist_id, hist_id) += init_cum_hist_mat(init_cum_hist_id, pix_id) * curr_hist_mat(hist_id, pix_id);
						init_cum_joint_hist_grad(_linear_idx(init_cum_hist_id, hist_id), pix_id) = init_cum_hist_grad(init_cum_hist_id, pix_id) * curr_hist_mat(hist_id, pix_id);
						++init_cum_hist_id;
					}
				}
			}
		}
	});

	init_cum_joint_hist *= hist_norm_mult;
	curr_hist *= hist_norm_mult;

	init_cum_joint_hist_log = init_cum_joint_hist.array().log();
	curr_hist_log = curr_hist.array().log();

	for(int id1 = 0; id1 < params.n_bins; id1++){
		for(int id2 = 0; id2 < params.n_bins; id2++){
			init_ccre_log_term(id1, id2) = init_cum_joint_hist_log(id1, id2) - curr_hist_log(id1) - init_cum_hist_log(id2);
		}
	}
	for(int pix = 0; pix < n_pix; pix++) {
		init_grad(pix) = 0;
		for(int id1 = _curr_bspl_ids(pix, 0); id1 <= _curr_bspl_ids(pix, 1); id1++) {
			for(int id2 = _init_bspl_ids(pix, 0); id2 <= _init_bspl_ids(pix, 1); id2++) {
				init_grad(pix) += init_cum_joint_hist_grad(_linear_idx(id1, id2), pix) * init_ccre_log_term(id1, id2);
			}
		}
	}
}
void  CCRE::cmptSymInitSimHessian(MatrixXd &hessian, const MatrixXd &init_pix_jacobian){
	assert(hessian.rows() == hessian.cols() && hessian.cols() == init_pix_jacobian.cols());
	assert(init_pix_jacobian.rows() == n_pix);

	int ssm_state_size = init_pix_jacobian.cols();
	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	joint_hist_jacobian.setZero();
	hessian.setZero();

	parallel_for(tbb::blocked_range<size_t>(0, params.n_blocks),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id <= r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				double hist_hess_term = 0;
				for(int t = _init_bspl_ids(pix_id, 0); t <= _init_bspl_ids(pix_id, 1); t++) {
					double inner_term = 0;
					for(int r = _curr_bspl_ids(pix_id, 0); r <= _curr_bspl_ids(pix_id, 1); r++) {
						int idx = _linear_idx(r, t);
						joint_hist_jacobian.row(idx) += init_cum_joint_hist_grad(idx, pix_id)*init_pix_jacobian.row(pix_id);
						inner_term += curr_hist_mat(r, pix_id) * init_ccre_log_term(r, t);
					}
					hist_hess_term += init_cum_hist_hess(t, pix_id)*inner_term;
				}
				hessian += hist_hess_term * init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id);
			}
		}
	});

	for(int r = 0; r < params.n_bins; r++){
		for(int t = 0; t < params.n_bins; t++){
			int idx = _linear_idx(r, t);
			double hist_factor = (1.0 / init_cum_joint_hist(r, t)) - (1.0 / init_cum_hist(t));
			hessian += joint_hist_jacobian.row(idx).transpose() * joint_hist_jacobian.row(idx) * hist_factor;
		}
	}
}

//-----------------------------------functor support-----------------------------------//

void CCRE::updateDistFeat(double* feat_addr){
	MatrixXdMr cum_hist_mat((double*)feat_addr, 9, static_n_pix);
	parallel_for(tbb::blocked_range<size_t>(0, static_n_pix),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				int pix_val_floor = static_cast<int>(curr_pix_vals(pix_id));
				double pix_diff = _std_bspl_ids(pix_val_floor, 0) - curr_pix_vals(pix_id);
				cum_hist_mat(0, pix_id) = pix_val_floor;
				cum_hist_mat(1, pix_id) = utils::cumBSpl3(pix_diff);
				cum_hist_mat(5, pix_id) = utils::bSpl3(pix_diff++);

				cum_hist_mat(2, pix_id) = utils::cumBSpl3(pix_diff);
				cum_hist_mat(6, pix_id) = utils::bSpl3(pix_diff++);

				cum_hist_mat(3, pix_id) = utils::cumBSpl3(pix_diff);
				cum_hist_mat(7, pix_id) = utils::bSpl3(pix_diff++);

				cum_hist_mat(4, pix_id) = utils::cumBSpl3(pix_diff);
				cum_hist_mat(8, pix_id) = utils::bSpl3(pix_diff);
			}
		}
	});
}
double CCRE::operator()(const double* hist1_mat_addr, const double* hist2_mat_addr,
	size_t hist_mat_size, double worst_dist) const{

	//printf("hist_mat_size: %ld\n", hist_mat_size);
	//printf("feat_size: %d\n", feat_size);

	assert(hist_mat_size == feat_size);

	VectorXd cum_hist(static_params.n_bins);
	VectorXd hist(static_params.n_bins);
	MatrixXd cum_joint_hist(static_params.n_bins, static_n_pix);

	cum_hist.fill(hist_pre_seed);
	hist.fill(hist_pre_seed);
	cum_joint_hist.fill(static_params.pre_seed);

	MatrixXdMr cum_hist_mat((double*)hist1_mat_addr, 9, static_n_pix);
	MatrixXdMr hist_mat((double*)hist2_mat_addr, 9, static_n_pix);

	//utils::printMatrixToFile(hist1_mat, "hist1_mat", "log/ccre_log.txt");
	//utils::printMatrixToFile(hist2_mat, "hist2_mat", "log/ccre_log.txt");


	parallel_for(tbb::blocked_range<size_t>(0, static_n_pix),
		[&](const tbb::blocked_range<size_t>& r){
		for(size_t block_id = r.begin(); block_id != r.end(); ++block_id){
			for(int pix_id = block_extents(block_id, 0); pix_id <= block_extents(block_id, 1); ++pix_id){
				int pix1_floor = static_cast<int>(cum_hist_mat(0, pix_id));
				int pix2_floor = static_cast<int>(hist_mat(0, pix_id));

				//printf("pix1_floor: %d\n", pix1_floor);
				//if(pix2_floor >= static_params.n_bins){
				//	utils::printMatrixToFile(hist2_mat, "hist2_mat", "log/ccre_log.txt");
				//	printf("pix2_floor: %d\n", pix2_floor);
				//}
				int bspl_id11 = _std_bspl_ids(pix1_floor, 0);
				int bspl_id12 = bspl_id11 + 1, bspl_id13 = bspl_id11 + 2, bspl_id14 = bspl_id11 + 3;
				int bspl_id21 = _std_bspl_ids(pix2_floor, 0);
				int bspl_id22 = bspl_id21 + 1, bspl_id23 = bspl_id21 + 2, bspl_id24 = bspl_id21 + 3;

				cum_hist(bspl_id11) += cum_hist_mat(1, pix_id);
				cum_hist(bspl_id12) += cum_hist_mat(2, pix_id);
				cum_hist(bspl_id13) += cum_hist_mat(3, pix_id);
				cum_hist(bspl_id14) += cum_hist_mat(4, pix_id);

				hist(bspl_id21) += hist_mat(5, pix_id);
				hist(bspl_id22) += hist_mat(6, pix_id);
				hist(bspl_id23) += hist_mat(7, pix_id);
				hist(bspl_id24) += hist_mat(8, pix_id);

				cum_joint_hist(bspl_id11, bspl_id21) += cum_hist_mat(1, pix_id) * hist_mat(5, pix_id);
				cum_joint_hist(bspl_id12, bspl_id21) += cum_hist_mat(2, pix_id) * hist_mat(5, pix_id);
				cum_joint_hist(bspl_id13, bspl_id21) += cum_hist_mat(3, pix_id) * hist_mat(5, pix_id);
				cum_joint_hist(bspl_id14, bspl_id21) += cum_hist_mat(4, pix_id) * hist_mat(5, pix_id);

				cum_joint_hist(bspl_id11, bspl_id22) += cum_hist_mat(1, pix_id) * hist_mat(6, pix_id);
				cum_joint_hist(bspl_id12, bspl_id22) += cum_hist_mat(2, pix_id) * hist_mat(6, pix_id);
				cum_joint_hist(bspl_id13, bspl_id22) += cum_hist_mat(3, pix_id) * hist_mat(6, pix_id);
				cum_joint_hist(bspl_id14, bspl_id22) += cum_hist_mat(4, pix_id) * hist_mat(6, pix_id);

				cum_joint_hist(bspl_id11, bspl_id23) += cum_hist_mat(1, pix_id) * hist_mat(7, pix_id);
				cum_joint_hist(bspl_id12, bspl_id23) += cum_hist_mat(2, pix_id) * hist_mat(7, pix_id);
				cum_joint_hist(bspl_id13, bspl_id23) += cum_hist_mat(3, pix_id) * hist_mat(7, pix_id);
				cum_joint_hist(bspl_id14, bspl_id23) += cum_hist_mat(4, pix_id) * hist_mat(7, pix_id);

				cum_joint_hist(bspl_id11, bspl_id24) += cum_hist_mat(1, pix_id) * hist_mat(8, pix_id);
				cum_joint_hist(bspl_id12, bspl_id24) += cum_hist_mat(2, pix_id) * hist_mat(8, pix_id);
				cum_joint_hist(bspl_id13, bspl_id24) += cum_hist_mat(3, pix_id) * hist_mat(8, pix_id);
				cum_joint_hist(bspl_id14, bspl_id24) += cum_hist_mat(4, pix_id) * hist_mat(8, pix_id);
			}
		}
	});

	ResultType result = 0;
	for(int id1 = 0; id1 < static_params.n_bins; id1++){
		for(int id2 = 0; id2 < static_params.n_bins; id2++){
			result -= cum_joint_hist(id1, id2) * (log(cum_joint_hist(id1, id2) / (cum_hist(id1) * hist(id2))) - log_hist_norm_mult);
		}
	}
	return result;
}

_MTF_END_NAMESPACE

