#include "mtf/SSM/Translation.h"
#include "mtf/SSM/SSMEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"

#ifndef CV_LMEDS
#define CV_LMEDS 4
#endif
#ifndef CV_RANSAC
#define CV_RANSAC 8
#endif

_MTF_BEGIN_NAMESPACE

TranslationParams::TranslationParams(const SSMParams *ssm_params,
bool _debug_mode) :
SSMParams(ssm_params),
debug_mode(_debug_mode){}

TranslationParams::TranslationParams(const TranslationParams *params) :
SSMParams(params),
debug_mode(TRANS_DEBUG_MODE){
	if(params){
		debug_mode = params->debug_mode;
	}
}

Translation::Translation(
const ParamType *_params) : ProjectiveBase(_params),
params(_params){

	printf("\n");
	printf("Using Translation SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "translation";
	state_size = 2;

	curr_state.resize(state_size);

	identity_jacobian = true;
}

void Translation::setCorners(const CornersT& corners){
	curr_corners = corners;
	getPtsFromCorners(curr_warp, curr_pts, curr_pts_hm, curr_corners);

	init_corners = curr_corners;
	init_pts = curr_pts;

	curr_warp = Matrix3d::Identity();
	curr_state.fill(0);
}

void Translation::setState(const VectorXd &ssm_state){
	validate_ssm_state(ssm_state);
	curr_state = ssm_state;
	getWarpFromState(curr_warp, curr_state);

	curr_pts = init_pts.colwise() + curr_state;
	curr_corners = init_corners.colwise() + curr_state;
}

void Translation::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	curr_state += state_update;
	curr_warp(0, 2) = curr_state(0);
	curr_warp(1, 2) = curr_state(1);

	curr_pts = curr_pts.colwise() + state_update;
	curr_corners = curr_corners.colwise() + state_update;
}

void Translation::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	warp_mat.setIdentity();
	warp_mat(0, 2) = ssm_state(0);
	warp_mat(1, 2) = ssm_state(1);
}

void Translation::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& warp_mat){
	VALIDATE_TRANS_WARP(warp_mat);

	state_vec(0) = warp_mat(0, 2);
	state_vec(1) = warp_mat(1, 2);
}

void Translation::invertState(VectorXd& inv_state, const VectorXd& state){
	inv_state = -state;
}

void Translation::updateGradPts(double grad_eps){

	for(int pix_id = 0; pix_id < n_pts; pix_id++){
		grad_pts(0, pix_id) = curr_pts(0, pix_id) + grad_eps;
		grad_pts(1, pix_id) = curr_pts(1, pix_id);

		grad_pts(2, pix_id) = curr_pts(0, pix_id) - grad_eps;
		grad_pts(3, pix_id) = curr_pts(1, pix_id);

		grad_pts(4, pix_id) = curr_pts(0, pix_id);
		grad_pts(5, pix_id) = curr_pts(1, pix_id) + grad_eps;

		grad_pts(6, pix_id) = curr_pts(0, pix_id);
		grad_pts(7, pix_id) = curr_pts(1, pix_id) - grad_eps;
	}
}

void Translation::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;

	for(int pix_id = 0; pix_id < n_pts; pix_id++){

		hess_pts(0, pix_id) = curr_pts(0, pix_id) + hess_eps2;
		hess_pts(1, pix_id) = curr_pts(1, pix_id);

		hess_pts(2, pix_id) = curr_pts(0, pix_id) - hess_eps2;
		hess_pts(3, pix_id) = curr_pts(1, pix_id);

		hess_pts(4, pix_id) = curr_pts(0, pix_id);
		hess_pts(5, pix_id) = curr_pts(1, pix_id) + hess_eps2;

		hess_pts(6, pix_id) = curr_pts(0, pix_id);
		hess_pts(7, pix_id) = curr_pts(1, pix_id) - hess_eps2;

		hess_pts(8, pix_id) = curr_pts(0, pix_id) + hess_eps;
		hess_pts(9, pix_id) = curr_pts(1, pix_id) + hess_eps;

		hess_pts(10, pix_id) = curr_pts(0, pix_id) - hess_eps;
		hess_pts(11, pix_id) = curr_pts(1, pix_id) - hess_eps;

		hess_pts(12, pix_id) = curr_pts(0, pix_id) + hess_eps;
		hess_pts(13, pix_id) = curr_pts(1, pix_id) - hess_eps;

		hess_pts(14, pix_id) = curr_pts(0, pix_id) - hess_eps;
		hess_pts(15, pix_id) = curr_pts(1, pix_id) + hess_eps;
	}
}

void Translation::applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
	const VectorXd &state_update){
	warped_corners = orig_corners.colwise() + state_update;
}
void Translation::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
	const VectorXd &state_update){
	warped_pts = orig_pts.colwise() + state_update;
}

void Translation::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);

	Vector2d out_centroid = out_corners.rowwise().mean();
	Vector2d in_centroid = in_corners.rowwise().mean();
	state_update = out_centroid - in_centroid;
}

// use Random Walk model to generate perturbed sample
void Translation::compositionalRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	generatePerturbation(state_perturbation);
	perturbed_state = base_state + state_perturbation;
}
// use first order Auto Regressive model to generate perturbed sample
void Translation::compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
	const VectorXd &base_state, const VectorXd &base_ar, double a){
	generatePerturbation(state_perturbation);
	perturbed_state = base_state + base_ar + state_perturbation;
	perturbed_ar = a*(perturbed_state - base_state);
}

void Translation::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	assert(in_pts.size() == out_pts.size() && in_pts.size() == mask.size());
	if(est_params.max_iters > 0){
		cv::Mat trans_params = estimateTranslation(in_pts, out_pts, mask, est_params);	
		state_update(0) = trans_params.at<double>(0, 0);
		state_update(1) = trans_params.at<double>(0, 1);
	} else{
		int n_pts = in_pts.size();
		vector<cv::Point2f> diff_between_pts(n_pts);
		vector<uchar> test_mask(n_pts);
		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			diff_between_pts[pt_id].x = out_pts[pt_id].x - in_pts[pt_id].x;
			diff_between_pts[pt_id].y = out_pts[pt_id].y - in_pts[pt_id].y;
		}
		int max_inliers = -1, best_pt_id = -1;
		switch(est_params.method){
		case EstType::LeastMedian:
			throw std::domain_error("translation::Least median estimator is not implemented yet");
		case EstType::RANSAC:
			for(int test_pt_id = 0; test_pt_id < n_pts; ++test_pt_id){
				float tx = out_pts[test_pt_id].x - in_pts[test_pt_id].x;
				float ty = out_pts[test_pt_id].y - in_pts[test_pt_id].y;
				int n_inliers = 0;
				for(int pt_id = 0; pt_id < n_pts; ++pt_id){
					float dx = diff_between_pts[pt_id].x - diff_between_pts[test_pt_id].x;
					float dy = diff_between_pts[pt_id].y - diff_between_pts[test_pt_id].y;
					float err = dx*dx + dy*dy;
					n_inliers += test_mask[pt_id] = err <= est_params.ransac_reproj_thresh;
				}
				if(n_inliers > max_inliers){
					max_inliers = n_inliers;
					best_pt_id = test_pt_id;
					memcpy(mask.data(), test_mask.data(), sizeof(uchar)*n_pts);
				}
			}
			break;
		default:
			std::fill(mask.begin(), mask.end(), 1);
		}

		state_update[0] = 0, state_update[1] = 0;
		int pts_found = 0;
		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			if(!mask[pt_id]){ continue; }
			++pts_found;
			state_update[0] += (diff_between_pts[pt_id].x - state_update[0]) / pts_found;
			state_update[1] += (diff_between_pts[pt_id].y - state_update[1]) / pts_found;
		}
	}
}


_MTF_END_NAMESPACE

