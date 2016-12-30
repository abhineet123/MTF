#ifndef MTF_PARAMETERS_H
#define MTF_PARAMETERS_H

#define SRC_IMG 'j'
#define SRC_VID 'm'
#define SRC_USB_CAM 'u'
#define SRC_FW_CAM 'f'
#define SRC_DISK 'd'


#define VID_FNAME "nl_bookI_s3"
#define VID_FMT "mpg"

#define IMG_FOLDER "nl_bookI_s3"
#define IMG_FMT "jpg"

#define ROOT_FOLDER "../../Datasets/TMT"

#define FW_DEV_NAME "firewire_cam"
#define FW_DEV_PATH "/dev/fw1"
#define FW_DEV_FMT "i1V1f7o3r150"

#define USB_DEV_NAME "usb_cam"
#define USB_DEV_PATH "/dev/video0"
#define USB_DEV_FMT "I1B4r0N0"

#define CV_CAM_ID 0

#define CASCADE_MAX_TRACKERS 10

#define parse_param(param_name, param_func)\
	do{if(!strcmp(arg_name, #param_name)){\
		param_name = param_func(arg_val);\
		return;\
			}} while(0)
#define parse_param_vec(param_name, param_func)\
	do{if(!strcmp(arg_name, #param_name)){\
		param_name.push_back(param_func(arg_val));\
		return;\
				}} while(0)

#include <cstddef>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "opencv2/core/core.hpp"
#include "mtf/Macros/common.h"

#include "datasets.h"

namespace mtf{
	namespace params{
		// folder where the config files mtf.cfg, modules.cfg and multi.cfg are located
		std::string config_dir = "Config";

		/* default parameters */
		int source_id = 0;
		int actor_id = 0;
		int n_trackers = 1;
		char pipeline = 'c';
		char img_source = 'j';
		double img_resize_factor = 1.0;
		int buffer_count = 10;
		int buffer_id = 0;

		//! flags
		int show_xv_window = 0;
		int pause_after_frame = 0;
		bool print_corners = false;
		int show_corner_ids = 0;
		int show_cv_window = 1;
		int show_ground_truth = 0;
		int reinit_gt_from_bin = 1;
		int read_obj_from_file = 0;
		int read_obj_from_gt = 1;
		bool sel_quad_obj = false;
		int line_thickness = 2;
		int record_frames = 0;
		int write_frame_data = 0;
		bool gt_write_ssm_params = false;
		int write_tracking_data = 0;
		bool overwrite_gt = 0;
		int write_pts = 1;
		int show_warped_img = 0;
		int show_proc_img = 0;
		int write_tracker_states = 0;
		int write_objs = 0;
		char rec_source = 'u';
		std::string rec_seq_suffix;
		int rec_fps = 24;


		bool reinit_at_each_frame = false;
		bool reset_at_each_frame = false;
		bool reset_to_init = false;
		int reinit_on_failure = false;
		double reinit_err_thresh = 5.0;
		int reinit_frame_skip = 5;
		bool reinit_with_new_obj = false;

		bool use_reinit_gt = false;
		bool use_opt_gt = false;
		std::string opt_gt_ssm = "2";
		int debug_mode = 0;
		int reset_template = 0;

		std::string read_obj_fname = "sel_objs/selected_objects.txt";
		std::string write_obj_fname = "sel_objs/selected_objects.txt";
		std::string tracking_data_fname;
		std::string record_frames_fname, record_frames_dir;

		std::string db_root_path = "../../../Datasets";
		std::string actor = "TMT";
		std::string source_path = db_root_path + "/" + actor;
		std::string source_name = "nl_bookI_s3";
		std::string source_fmt = "jpg";

		//! for Xvision trackers
		int steps_per_frame = 1;
		int patch_size = 0;

		//char frame_dir[200];
		//char pts_dir[200];
		//char states_dir[200];

		/* only for pyramidal trackers */
		int no_of_levels = 2;
		double scale = 0.5;
		/* only for color tracker; enabling it causes tracker to hang and crash */
		bool color_resample = false;

		int search_width = 50;
		int search_angle = 0;
		int line_width = 4;

		/* for xvision grid tracker */
		int xvg_grid_size_x = 4;
		int xvg_grid_size_y = -1;
		int xv_tracker_type = 't';
		int xvg_reset_pos = 0;
		int xvg_reset_wts = 0;
		double xvg_sel_reset_thresh = 1.0;
		int xvg_pause_after_line = 0;
		int xvg_show_tracked_pts = 0;
		int xvg_use_constant_slope = 0;
		int xvg_use_ls = 0;
		int xvg_update_wts = 0;
		double xvg_inter_alpha_thresh = 0.10;
		double xvg_intra_alpha_thresh = 0.05;
		int xvg_adjust_grid = 0;
		int xvg_adjust_lines = 1;

		//! for MTF
		unsigned int mtf_res = 50;
		unsigned int resx = 50;
		unsigned int resy = 50;
		int init_frame_id = 0;
		int start_frame_id = 0;
		int end_frame_id = 0;
		int frame_gap = 1;
		int max_iters = 10;
		double epsilon = 0.01;
		char* mtf_sm = "esm";
		char* mtf_am = "ssd";
		char* mtf_ssm = "8";
		char* mtf_ilm = "0";
		bool enable_nt = false;
		bool invalid_state_check = true;
		double invalid_state_err_thresh = 0;

		bool ic_update_ssm = true;
		bool ic_chained_warp = true;
		int ic_hess_type = 0;

		bool fc_chained_warp = false;
		int fc_hess_type = 1;
		bool fc_write_ssm_updates = false;
		bool fc_show_grid = false;
		bool fc_show_patch = false;
		double fc_patch_resize_factor = 1.0;
		bool fc_debug_mode = false;

		int fa_hess_type = 1;
		bool fa_show_grid = false;
		bool fa_show_patch = false;
		double fa_patch_resize_factor = 1.0;
		bool fa_write_frames = false;

		int ia_hess_type = 1;

		int esm_jac_type = 1;
		int esm_hess_type = 2;
		bool esm_chained_warp = false;

		bool sec_ord_hess = false;
		bool leven_marq = false;
		double lm_delta_init = 0.01;
		double lm_delta_update = 10;

		bool enable_learning = false;


		bool ssd_show_template = false;
		double ssd_forgetting_factor = 0.5;

		double nssd_norm_pix_min = 0.0;
		double nssd_norm_pix_max = 1.0;

		double zncc_likelihood_alpha = 50;


		double res_from_size = 0;
		int show_tracking_error = 0;
		int write_tracking_error = 0;

		int tracking_err_type = 0;
		bool show_jaccard_error = false;
		double grad_eps = 1e-8;
		double hess_eps = 1.0;
		double likelihood_alpha = 1.0;
		double likelihood_beta = 0.0;
		bool dist_from_likelihood = false;

		std::string pre_proc_type = "gauss";
		//! perform histogram equalization during pre processing;
		bool pre_proc_hist_eq = false;
		int gauss_kernel_size = 5;
		double gauss_sigma_x = 3;
		double gauss_sigma_y = 3;
		int med_kernel_size = 5;
		int box_kernel_size = 5;
		int bil_diameter = 5;
		double bil_sigma_col = 15;
		double bil_sigma_space = 15;

		//! Affine
		int aff_normalized_init = 0;
		int aff_pt_based_sampling = 0;

		//! Homograhy
		int hom_normalized_init = 0;
		bool hom_corner_based_sampling = true;

		//! Lie Homograhy
		int lhom_normalized_init = 0;
		double lhom_grad_eps = 1e-8;

		//! Corner based Homograhy
		int chom_normalized_init = 0;
		double chom_grad_eps = 1e-8;

		//! Similitude
		bool sim_normalized_init = false;
		bool sim_geom_sampling = true;
		int sim_pt_based_sampling = 0;
		int sim_n_model_pts = 2;

		//! Isometry
		int iso_pt_based_sampling = 0;

		//! SL3
		int sl3_normalized_init = 0;
		bool sl3_iterative_sample_mean = true;
		int sl3_sample_mean_max_iters = 10;
		double sl3_sample_mean_eps = 1e-4;
		bool sl3_debug_mode = 0;

		//! Spline SSM
		int spl_control_size = 10;
		double	spl_control_overlap = 1;
		int	spl_interp_type = 0;
		bool spl_static_wts = true;
		bool spl_debug_mode = 0;

		//! SCV and RSCV
		int scv_hist_type = 0;
		bool scv_use_bspl = 0;
		int scv_n_bins = 256;
		double scv_preseed = 0;
		bool scv_pou = 1;
		bool scv_weighted_mapping = 1;
		bool scv_mapped_gradient = 1;
		bool scv_affine_mapping = 0;
		bool scv_once_per_frame = 0;
		bool scv_approx_dist_feat = true;
		double scv_likelihood_alpha = 0;

		//! LSCV and LRSCV
		int lscv_sub_regions = 3;
		int lscv_spacing = 10;
		bool lscv_show_subregions = false;

		//! LKLD
		int lkld_n_bins = 8;
		double lkld_pre_seed = 0.1;
		bool lkld_pou = 1;
		int lkld_sub_regions = 0;
		int lkld_spacing = 1;

		//! NCC
		bool ncc_fast_hess = false;
		double ncc_likelihood_alpha = 50;

		//! SPSS
		double spss_k = 0.01;
		double spss_likelihood_alpha = 50;

		//! SSIM
		int ssim_pix_proc_type = 0;
		double ssim_k1 = 0.01;
		double ssim_k2 = 0.03;
		double ssim_likelihood_alpha = 50;

		//! Sum of AMs
		std::string sum_am1, sum_am2;

		char *pix_mapper = nullptr;

		//! MI & CCRE
		int mi_n_bins = 8;
		double mi_pre_seed = 10;
		bool mi_pou = false;
		double mi_likelihood_alpha = 50;

		//! CCRE
		int ccre_n_bins = 8;
		double ccre_pre_seed = 10;
		bool ccre_pou = false;
		bool ccre_symmetrical_grad = false;
		int ccre_n_blocks = 0;
		double ccre_likelihood_alpha = 50;

		//!NGF
		double ngf_eta = 5.0;
		bool ngf_use_ssd = false;

		//! NN
		int nn_max_iters = 10;
		int nn_n_samples = 1000;
		vectori nn_ssm_sigma_ids;
		vectori nn_ssm_mean_ids;
		double nn_corner_sigma_d = 0.04;
		double nn_corner_sigma_t = 0.06;
		vectord nn_pix_sigma;
		int nn_n_trees = 6;
		int nn_n_checks = 50;
		double nn_ssm_sigma_prec = 1.1;
		int nn_index_type = 1;
		int nn_search_type = 0;
		bool nn_additive_update = false;
		int nn_show_samples = 0;
		int nn_add_points = 0;
		int nn_remove_points = 0;
		bool nn_save_index = false;
		bool nn_load_index = false;
		int nn_saved_index_fid = 0;
		//! GNN
		int nn_gnn_degree = 250;
		int nn_gnn_max_steps = 10;
		int nn_gnn_cmpt_dist_thresh = 10000;
		bool nn_gnn_random_start = false;
		bool nn_gnn_verbose = false;
		int nn_fgnn_index_type = 0;
		//! FLANN specific params
		int nn_srch_checks = 32;
		float nn_srch_eps = 0.0;
		bool nn_srch_sorted = true;
		int nn_srch_max_neighbors = -1;
		int nn_srch_cores = 1;
		bool nn_srch_matrices_in_gpu_ram = false;
		int nn_srch_use_heap = 2;
		int nn_kdt_trees = 6;
		int nn_km_branching = 32;
		int nn_km_iterations = 11;
		int nn_km_centers_init = 0;
		float nn_km_cb_index = 0.2;
		int nn_kdts_leaf_max_size = 10;
		int nn_kdtc_leaf_max_size = 64;
		int nn_hc_branching = 32;
		int nn_hc_trees = 4;
		int nn_hc_leaf_max_size = 100;
		int nn_hc_centers_init = 0;
		float nn_auto_target_precision = 0.9;
		float nn_auto_build_weight = 0.01;
		float nn_auto_memory_weight = 0;
		float nn_auto_sample_fraction = 0.1;

		// Multi Layer Nearest Filter
		int nnk_n_layers;
		vectorvi nnk_ssm_sigma_ids;

		//! Regression Network
		int rg_max_iters = 10;
		int rg_n_samples = 1000;
		vectori rg_ssm_sigma_ids;
		vectori rg_ssm_mean_ids;
		vectord rg_pix_sigma;
		bool rg_additive_update = false;
		int rg_show_samples = 0;
		int rg_add_points = 0;
		int rg_remove_points = 0;
		bool rg_save_index = false;
		bool rg_load_index = false;
		int rg_saved_index_fid = 0;
		int rg_nepochs = 10;
		int rg_bs = 128;
		bool rg_preproc = true;
		char *rg_solver = nullptr;
		char *rg_train = nullptr;
		char *rg_mean = nullptr;
		bool rg_dbg = false;
		bool rg_pretrained = false;

		//! RIU AM
		double riu_likelihood_alpha = 50.0;

		//! Gradient Descent
		double gd_learning_rate = 0.1;

		//! Gain and Bias Illumination Model
		bool gb_additive_update = false;

		//! Piecewise Gain and Bias Illumination Model
		bool pgb_additive_update = false;
		int pgb_sub_regions_x = 3;
		int pgb_sub_regions_y = 3;

		//! Radial Basis Function illumination model
		bool rbf_additive_update = false;
		int rbf_n_ctrl_pts_x = 3;
		int rbf_n_ctrl_pts_y = 3;

		//! Particle Filter
		int pf_max_iters = 1;
		int pf_n_particles = 100;
		int pf_dynamic_model = 1;
		int pf_update_type = 0;
		int pf_likelihood_func = 0;
		int pf_resampling_type = 0;
		int pf_mean_type = 1;
		bool pf_reset_to_mean = false;
		vectori pf_ssm_sigma_ids;
		vectori pf_ssm_mean_ids;
		bool pf_update_distr_wts = false;
		double pf_min_distr_wt = 0.1;
		double pf_measurement_sigma = 0.1;
		vectord pf_pix_sigma;
		int pf_show_particles = 0;
		bool pf_jacobian_as_sigma = false;
		bool pf_debug_mode = false;

		//! Multi Layer Particle Filter
		int pfk_n_layers;
		vectorvi pfk_ssm_sigma_ids;

		//! Gaussian parameters for sampling SSM parameters - used by NN, PF and RG
		vectorvd ssm_sigma;
		vectorvd ssm_mean;

		vectorvd am_sigma;
		vectorvd am_mean;

		//! Hierarchical SSM tracker
		char* hrch_sm = "iclk";
		char* hrch_am = "ssd";

		//! Cascade tracker
		int casc_n_trackers = 2;
		bool casc_enable_feedback = 1;
		bool casc_auto_reinit = false;
		double casc_reinit_err_thresh = 1.0;
		int casc_reinit_frame_gap = 1;

		//! Grid tracker
		char* grid_sm = "iclk";
		char* grid_am = "ssd";
		char* grid_ssm = "2";
		char* grid_ilm = "0";

		int grid_res = 10;
		int grid_patch_size = 10;
		int grid_patch_res = 0;
		int grid_reset_at_each_frame = 1;
		bool grid_dyn_patch_size = false;
		bool grid_patch_centroid_inside = true;
		bool grid_show_trackers = false;
		bool grid_show_tracker_edges = false;
		bool grid_use_tbb = true;
		//! OpenCV grid tracker
		int grid_pyramid_levels = 2;
		bool grid_use_min_eig_vals = 0;
		double grid_min_eig_thresh = 1e-4;

		bool grid_detect_keypoints = false;
		bool grid_rebuild_index = false;

		bool grid_use_const_grad = true;

		int sift_n_features = 0;
		int sift_n_octave_layers = 3;
		double sift_contrast_thresh = 0.04;
		double sift_edge_thresh = 10;
		double sift_sigma = 1.6;

		//! SSM Estimator
		int est_method = 0;
		double est_ransac_reproj_thresh = 10;
		int est_n_model_pts = 4;
		int est_max_iters = 2000;
		int est_max_subset_attempts = 300;
		bool est_use_boost_rng = false;
		double est_confidence = 0.995;
		bool est_refine = true;
		int est_lm_max_iters = 10;


		char* line_sm = "iclk";
		char* line_am = "ssd";
		char* line_ssm = "2";
		int line_grid_size = 5;
		int line_patch_size = 25;
		bool line_use_constant_slope = false;
		bool line_use_ls = false;
		double line_inter_alpha_thresh = 0.1;
		double line_intra_alpha_thresh = 0.05;
		bool line_reset_pos = false;
		bool line_reset_template = false;
		bool line_debug_mode = false;

		//! RKL Tracker
		char* rkl_sm = "iclk";
		bool rkl_enable_spi = true;
		bool rkl_enable_feedback = true;
		bool rkl_failure_detection = true;
		double rkl_failure_thresh = 15.0;

		//! Parallel Tracker
		int prl_n_trackers = 1;
		int prl_estimation_method = 0;
		bool prl_reset_to_mean = false;
		bool prl_auto_reinit = false;
		double prl_reinit_err_thresh = 1.0;
		int prl_reinit_frame_gap = 1;

		//! Pyramidal Tracker
		std::string pyr_sm = "fc";
		int pyr_no_of_levels = 3;
		double pyr_scale_factor = 0.50;
		bool pyr_scale_res = true;
		bool pyr_show_levels = false;

		//! MTF Diagnostics
		char* diag_am = "ssd";
		char* diag_ssm = "2";
		char* diag_ilm = "0";
		bool diag_3d = false;
		std::vector<int> diag_3d_ids = { 0, 1 };
		int diag_frame_gap = 0;
		double diag_range = 0;
		std::vector<double> diag_ssm_range;
		std::vector<double> diag_am_range;
		int diag_ssm_range_id = 0;
		int diag_am_range_id = 0;
		std::string diag_gen_norm = "000";// Norm,FeatNorm
		std::string diag_gen_jac = "000";// Std,ESM,Diff
		std::string diag_gen_hess = "0000";// Std,ESM,InitSelf,CurrSelf
		std::string diag_gen_hess2 = "0000";// Std2,ESM2,InitSelf2,CurrSelf2
		std::string diag_gen_hess_sum = "0000";// Std, Std2, Self, Self2
		std::string diag_gen_num = "000"; // Jac, Hess, NHess
		std::string diag_gen_ssm = "0";// ssm params

		bool diag_bin = true;
		bool diag_inv = true;
		bool diag_show_data = false;
		bool diag_show_corners = false;
		bool diag_show_patches = false;
		bool diag_verbose = false;

		double diag_grad_diff = 0.1;
		int diag_res = 50;
		int diag_update_type = 0;
		int diag_start_id = 0;
		int diag_end_id = 0;

		std::string diag_out_prefix;


		bool diag_enable_validation = false;
		double diag_validation_prec = 1e-20;

		bool esm_spi_enable = false;
		double esm_spi_thresh = 10;

		//! DSST
		double dsst_padding = 1;
		double dsst_sigma = 1.0 / 16;
		double dsst_scale_sigma = 1.0 / 4;
		double dsst_lambda = 1e-2;
		double dsst_learning_rate = 0.025;
		int dsst_number_scales = 33;
		int dsst_number_rots = 21;
		double dsst_scale_step = 1.02;
		double dsst_rot_step = 2;
		int dsst_resize_factor = 4;
		int dsst_is_scaling = 1;
		int dsst_is_rotating = 1;
		int dsst_bin_size = 1;

		//! KCF
		double kcf_padding; //extra area surrounding the target
		double kcf_lambda; //regularization
		double kcf_output_sigma_factor; //spatial bandwidth (proportional to target)
		double kcf_interp_factor; //linear interpolation factor for adaptation
		double kcf_kernel_sigma; //gaussian kernel bandwidth
		//! for scaling
		int kcf_number_scales;
		double kcf_scale_step;
		double kcf_scale_model_max_area;
		double kcf_scale_sigma_factor;
		double kcf_scale_learning_rate;
		bool kcf_enableScaling;
		int kcf_resize_factor;

		int mil_algorithm = 100;
		int mil_num_classifiers = 100;
		float mil_overlap = 0.99;
		float mil_search_factor = 2.0;
		float mil_pos_radius_train = 4.0;
		int mil_neg_num_train = 65;
		int mil_num_features = 250;

		//! CMT
		bool cmt_estimate_scale = true;
		bool cmt_estimate_rotation = false;
		char* cmt_feat_detector = "FAST";
		char* cmt_desc_extractor = "BRISK";
		double cmt_resize_factor = 0.5;

		//! TLD
		bool tld_tracker_enabled = true;
		bool tld_detector_enabled = true;
		bool tld_learning_enabled = true;
		bool tld_alternating = false;

		//! RCT
		int rct_min_n_rect = 2;
		int rct_max_n_rect = 4;
		int rct_n_feat = 50;
		int rct_rad_outer_pos = 4;
		int rct_rad_search_win = 25;
		double rct_learning_rate = 0.85;

		std::string strk_config_path = "Config/struck.cfg";

		//! ViSP Template Tracker
		char* visp_sm = "fclk";
		char* visp_am = "ssd";
		char* visp_ssm = "8";
		int visp_max_iters = 30;
		int visp_res = 50;
		double visp_lambda = 0.001;
		double visp_thresh_grad = 60;
		int visp_pyr_n_levels = 0;
		int visp_pyr_level_to_stop = 1;
		//! ViSP Pipeline
		int visp_fw_res = 0;
		int visp_fw_fps = 0;
		int visp_usb_res = 0;
		int visp_usb_fps = 0;
		int visp_usb_n_buffers = 3;


		//! PFSL3
		int pfsl3_p_x = 40;
		int pfsl3_p_y = 40;
		double pfsl3_rot = 0;
		double pfsl3_ncc_std = 0.1;
		double pfsl3_pca_std = 10;
		std::vector<double> pfsl3_state_std;
		double pfsl3_ar_p = 0.5;
		int pfsl3_n = 40;
		int pfsl3_n_c = 10;
		int pfsl3_n_iter = 5;
		int pfsl3_sampling = 0;
		int pfsl3_capture = 0;
		int pfsl3_mean_check = 0;
		int pfsl3_outlier_flag = 0;
		int pfsl3_len = 100;
		int pfsl3_init_size = 15;
		int pfsl3_update_period = 5;
		float pfsl3_ff = 0.99;
		double pfsl3_basis_thr = 0.95;
		int pfsl3_max_num_basis = 30;
		int pfsl3_max_num_used_basis = 10;
		bool pfsl3_show_weights = false;
		bool pfsl3_show_templates = false;
		bool pfsl3_debug_mode = false;

		//! GOTURN
		bool gtrn_do_train = true;
		int gtrn_gpu_id = 0;
		bool gtrn_show_intermediate_output = false;
		std::string gtrn_model_file = "Data/GOTURN/tracker.prototxt";
		std::string gtrn_trained_file = "Data/GOTURN/solver.prototxt";

		//! DFT
		float dft_res_to_l = 1e-10;
		float dft_p_to_l = 5e-5;
		int dft_max_iter = 50;
		int dft_max_iter_single_level = 10;
		std::vector<float> dft_pyramid_smoothing_variance = { 7 };
		float dft_presmoothing_variance = 1;
		int dft_n_control_points_on_edge = 25;
		bool dft_b_adaptative_choice_of_points = 0;
		bool dft_b_normalize_descriptors = 0;
		int dft_optimization_type = 2;

		int frg_n_bins = 16;
		int frg_search_margin = 7;
		int frg_hist_cmp_metric = 3;
		double frg_resize_factor = 0.5;
		bool frg_show_window = false;

		//! PCA Patch Extraction
		std::vector<int> extracted_frame_ids = { 0, 1, 2, 3, 4 };
		int extraction_id = 0;
		// PCA
		int pca_n_eigenvec = 16;
		int pca_batchsize = 5;
		float pca_f_factor = 0.95;
		bool pca_show_basis = false;

		//! FMaps
		int dfm_nfmaps = 1;
		char* dfm_layer_name = "conv1";
		int dfm_vis = 0;
		int dfm_zncc = 0;
		char *dfm_model_f_name = "../../../VGG_Models/VGG_deploy.prototxt";
		char *dfm_params_f_name = "../../../VGG_Models/VGG_CNN_F.caffemodel";
		char *dfm_mean_f_name = "../../../VGG_Models/VGG_mean.binaryproto";

		//! HACLK
		//std::vector<cv::Mat> conv_corners;
		std::string  syn_ssm = "c8";
		std::string  syn_ilm = "0";
		int syn_frame_id = 0;
		bool syn_grayscale_img = false;
		bool syn_continuous_warping = true;
		vectori syn_ssm_sigma_ids, syn_ssm_mean_ids;
		vectori syn_am_sigma_ids, syn_am_mean_ids;
		double syn_pix_sigma = 0;
		bool syn_am_on_obj = false;
		bool syn_warp_entire_image = false;
		int syn_background_type = 0;
		bool syn_use_inv_warp = false;
		std::string syn_out_suffix;
		int syn_n_frames = 1;
		bool syn_add_noise = true;
		double syn_noise_mean = 0.0;
		double syn_noise_sigma = 1.0;
		bool syn_save_as_video = false;
		int syn_video_fps = 24;
		int syn_jpg_quality = 100;
		bool syn_show_output = true;

		int mos_track_border = 100;
		int mos_border = 200;
		int mos_disp_width = 200;
		int mos_disp_height = 200;
		bool mos_show_grid = false;
		bool mos_use_write_mask = false;

		inline void split(const std::string &s, char delim, std::vector<std::string> &elems) {
			stringstream ss(s);
			std::string item;
			while(getline(ss, item, delim)) {
				if(!item.empty()){
					elems.push_back(item);
				}
			}
		}

		inline int readParams(std::vector<char*> &fargv, const char* fname = "mtf.cfg"){
			FILE *fid = fopen(fname, "r");
			if(!fid){
				printf("\n Parameter file: %s not found\n", fname);
				return 0;
			}
			// one extra entry at the beginning to maintain compatibilty with the C/C++
			// command line argument convention
			fargv.push_back(nullptr);
			int arg_id = 0;
			while(!feof(fid)){
				char *temp = new char[500];
				fgets(temp, 500, fid);
				strtok(temp, "\n");
				strtok(temp, "\r");
				if(strlen(temp) <= 1 || temp[0] == '#'){
					delete(temp);
					continue;
				}
				fargv.push_back(temp);
				++arg_id;
				//printf("arg %d: %s\n", arg_id, fargv[arg_id]);
			}
			fclose(fid);
			return arg_id + 1;
		}

		inline void processStringParam(std::string &str_out, const char* param){
			if(!strcmp(param, "#")){
				// use default value
				return;
			}
			str_out = std::string(param);
		}

		inline void processStringParam(char* &str_out, const char* param){
			//printf("-----------------------\n");
			//printf("param: %s\n", param);
			//printf("use_default: %c\n", use_default);
			if(!strcmp(param, "#")){
				// use default value
				return;
			}
			//if((strlen(param) == 2) && (param[0] == '0') && (param[1] == '0')){
			//	return;
			//}
			//if(str_out){ delete(str_out); }

			str_out = new char[strlen(param) + 1];
			strcpy(str_out, param);
			//printf("str_out: %s\n", str_out);
		}
		// convert a string of comma (or other specified character) separated values
		// into a vector of doubles
		inline vectord atof_arr(char *str, const char *sep = ","){
			char *param_str = strtok(str, sep);
			std::vector<double> param_arr;
			//printf("atof_arr::str: %s\n", str);
			//printf("atof_arr::param_arr:\n");
			while(param_str){
				double param_num = atof(param_str);
				param_arr.push_back(param_num);
				//printf("%f\t", param_num);
				param_str = strtok(nullptr, sep);
			}
			//printf("\n");
			//std::string str_temp(str);
			//for(int i = 0; i < str_temp.length(); ++i){
			//	if(str_temp[i] == sep)
			//		str_temp[i] = ' ';
			//}
			//printf("atof_arr::str_temp: %s\n", str_temp.c_str());
			//std::stringstream ss(str);
			//double temp;
			//while(ss >> temp)
			//	param_arr.push_back(temp);
			return param_arr;
		}
		inline vectorf atof32_arr(char *str, const char *sep = ","){
			char *param_str = strtok(str, sep);
			std::vector<float> param_arr;
			while(param_str){
				float param_num = atof(param_str);
				param_arr.push_back(param_num);
				param_str = strtok(nullptr, sep);
			}
			return param_arr;
		}
		inline vectori atoi_arr(char *str, const char *sep = ","){
			char *param_str = strtok(str, sep);
			std::vector<int> param_arr;
			//printf("atof_arr::str: %s\n", str);
			//printf("atof_arr::param_arr:\n");
			while(param_str){
				double param_num = atoi(param_str);
				param_arr.push_back(param_num);
				//printf("%f\t", param_num);
				param_str = strtok(nullptr, sep);
			}
			//printf("\n");
			//std::string str_temp(str);
			//for(int i = 0; i < str_temp.length(); ++i){
			//	if(str_temp[i] == sep)
			//		str_temp[i] = ' ';
			//}
			//printf("atof_arr::str_temp: %s\n", str_temp.c_str());
			//std::stringstream ss(str);
			//double temp;
			//while(ss >> temp)
			//	param_arr.push_back(temp);
			return param_arr;
		}
		inline char atoc(char *arg_val){ return arg_val[0]; }
		inline int atoc_i(char *arg_val){ return arg_val[0] - '0'; }

		inline void processAgrument(char *arg_name, char *arg_val,
			char *arg_prefix = nullptr){
			if(arg_val[0] == '#'){ return; }

			//parse_param(n_trackers, atoi);
			//parse_param(source_id, atoi);
			//parse_param(actor_id, atoi);
			//parse_param(pipeline, atoc);
			//parse_param(img_source, atoc);
			//parse_param(write_frame_data, atoc_i);
			//parse_param(write_tracking_data, atoc_i);
			//parse_param(tracking_data_fname, std::string);
			//parse_param(record_frames_fname, std::string);
			//parse_param(read_obj_fname, std::string);
			//parse_param(write_obj_fname, std::string);
			//parse_param(write_pts, atoc_i);
			//parse_param(show_cv_window, atoc_i);
			//parse_param(show_ground_truth, atoc_i);
			//parse_param(show_xv_window, atoc_i);
			//parse_param(read_obj_from_gt, atoi);
			//parse_param(sel_quad_obj, atoi);

			if(!strcmp(arg_name, "n_trackers")){
				n_trackers = atoi(arg_val);
			} else if(!strcmp(arg_name, "source_id")){
				source_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "actor_id")){
				actor_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "pipeline")){
				pipeline = arg_val[0];
			} else if(!strcmp(arg_name, "img_source")){
				img_source = arg_val[0];
			} else if(!strcmp(arg_name, "rec_source")){
				rec_source = arg_val[0];
			} else if(!strcmp(arg_name, "rec_fps")){
				rec_fps = atoi(arg_val);
			} else if(!strcmp(arg_name, "rec_seq_suffix")){
				rec_seq_suffix = std::string(arg_val);
			} else if(!strcmp(arg_name, "img_resize_factor")){
				img_resize_factor = atof(arg_val);
			} else if(!strcmp(arg_name, "n_trackers")){
				n_trackers = atoi(arg_val);
			} else if(!strcmp(arg_name, "write_frame_data")){
				write_frame_data = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "gt_write_ssm_params")){
				gt_write_ssm_params = atoi(arg_val);
			} else if(!strcmp(arg_name, "write_tracking_data")){
				write_tracking_data = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "overwrite_gt")){
				overwrite_gt = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "tracking_data_fname")){
				tracking_data_fname = std::string(arg_val);
			} else if(!strcmp(arg_name, "record_frames_fname")){
				record_frames_fname = std::string(arg_val);
			} else if(!strcmp(arg_name, "record_frames_dir")){
				record_frames_dir = std::string(arg_val);
			} else if(!strcmp(arg_name, "read_obj_fname")){
				read_obj_fname = std::string(arg_val);
			} else if(!strcmp(arg_name, "write_obj_fname")){
				write_obj_fname = std::string(arg_val);
			} else if(!strcmp(arg_name, "write_pts")){
				write_pts = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "show_corner_ids")){
				show_corner_ids = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "show_cv_window")){
				show_cv_window = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "show_ground_truth")){
				show_ground_truth = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "show_xv_window")){
				show_xv_window = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "read_obj_from_gt")){
				read_obj_from_gt = atoi(arg_val);
			} else if(!strcmp(arg_name, "sel_quad_obj")){
				sel_quad_obj = atoi(arg_val);
			} else if(!strcmp(arg_name, "line_thickness")){
				line_thickness = atoi(arg_val);
			} else if(!strcmp(arg_name, "reinit_gt_from_bin")){
				reinit_gt_from_bin = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "read_obj_from_file")){
				read_obj_from_file = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "write_objs")){
				write_objs = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "record_frames")){
				record_frames = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "source_name")){
				source_name = std::string(arg_val);
			} else if(!strcmp(arg_name, "source_path")){
				source_path = std::string(arg_val);
			} else if(!strcmp(arg_name, "patch_size")){
				patch_size = atoi(arg_val);
			} else if(!strcmp(arg_name, "source_fmt")){
				source_fmt = std::string(arg_val);
			} else if(!strcmp(arg_name, "db_root_path") || !strcmp(arg_name, "root_path")){
				db_root_path = std::string(arg_val);
			} else if(!strcmp(arg_name, "xvg_grid_size_x")){
				xvg_grid_size_x = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_grid_size_y")){
				xvg_grid_size_y = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_adjust_grid")){
				xvg_adjust_grid = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "xvg_adjust_lines")){
				xvg_adjust_lines = arg_val[0] - '0';
			} else if(!strcmp(arg_name, "xvg_reset_pos")){
				xvg_reset_pos = atoi(arg_val);
			} else if(!strcmp(arg_name, "reset_template")){
				reset_template = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_reset_wts")){
				xvg_reset_wts = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_pause_after_line")){
				xvg_pause_after_line = atoi(arg_val);
			} else if(!strcmp(arg_name, "debug_mode")){
				debug_mode = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_use_constant_slope")){
				xvg_use_constant_slope = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_use_ls")){
				xvg_use_ls = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_inter_alpha_thresh")){
				xvg_inter_alpha_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "xvg_intra_alpha_thresh")){
				xvg_intra_alpha_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "pause_after_frame")){
				pause_after_frame = atoi(arg_val);
			} else if(!strcmp(arg_name, "print_corners")){
				print_corners = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_show_tracked_pts")){
				xvg_show_tracked_pts = atoi(arg_val);
			} else if(!strcmp(arg_name, "show_warped_img")){
				show_warped_img = atoi(arg_val);
			} else if(!strcmp(arg_name, "show_proc_img")){
				show_proc_img = atoi(arg_val);
			} else if(!strcmp(arg_name, "write_tracker_states")){
				write_tracker_states = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_update_wts")){
				xvg_update_wts = atoi(arg_val);
			} else if(!strcmp(arg_name, "xvg_sel_reset_thresh")){
				xvg_sel_reset_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "res_from_size")){
				res_from_size = atof(arg_val);
			} else if(!strcmp(arg_name, "mtf_res")){
				mtf_res = atoi(arg_val);
			} else if(!strcmp(arg_name, "resx")){
				resx = atoi(arg_val);
			} else if(!strcmp(arg_name, "resy")){
				resy = atoi(arg_val);
			} else if(!strcmp(arg_name, "epsilon")){
				epsilon = atof(arg_val);
			} else if(!strcmp(arg_name, "max_iters")){
				max_iters = atoi(arg_val);
			} else if(!strcmp(arg_name, "mtf_sm")){
				processStringParam(mtf_sm, arg_val);
			} else if(!strcmp(arg_name, "mtf_am")){
				processStringParam(mtf_am, arg_val);
			} else if(!strcmp(arg_name, "mtf_ssm")){
				processStringParam(mtf_ssm, arg_val);
			} else if(!strcmp(arg_name, "mtf_ilm")){
				processStringParam(mtf_ilm, arg_val);
			} else if(!strcmp(arg_name, "enable_nt")){
				enable_nt = atoi(arg_val);
			} else if(!strcmp(arg_name, "invalid_state_err_thresh")){
				invalid_state_err_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "invalid_state_check")){
				invalid_state_check = atoi(arg_val);
			}

			//! ICLK
			else if(!strcmp(arg_name, "ic_update_ssm")){
				ic_update_ssm = atoi(arg_val);
			} else if(!strcmp(arg_name, "ic_chained_warp")){
				ic_chained_warp = atoi(arg_val);
			} else if(!strcmp(arg_name, "ic_hess_type")){
				ic_hess_type = atoi(arg_val);
			}
			//! FCLK
			else if(!strcmp(arg_name, "fc_chained_warp")){
				fc_chained_warp = atoi(arg_val);
			} else if(!strcmp(arg_name, "fc_hess_type")){
				fc_hess_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "fc_write_ssm_updates")){
				fc_write_ssm_updates = atoi(arg_val);
			} else if(!strcmp(arg_name, "fc_show_grid")){
				fc_show_grid = atoi(arg_val);
			} else if(!strcmp(arg_name, "fc_show_patch")){
				fc_show_patch = atoi(arg_val);
			} else if(!strcmp(arg_name, "fc_patch_resize_factor")){
				fc_patch_resize_factor = atoi(arg_val);
			} else if(!strcmp(arg_name, "fc_debug_mode")){
				fc_debug_mode = atoi(arg_val);
			}
			// IA/FA
			else if(!strcmp(arg_name, "ia_hess_type")){
				ia_hess_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "fa_hess_type")){
				fa_hess_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "fa_show_grid")){
				fa_show_grid = atoi(arg_val);
			} else if(!strcmp(arg_name, "fa_patch_resize_factor")){
				fa_patch_resize_factor = atof(arg_val);
			} else if(!strcmp(arg_name, "fa_show_patch")){
				fa_show_patch = atoi(arg_val);
			} else if(!strcmp(arg_name, "fa_write_frames")){
				fa_write_frames = atoi(arg_val);
			}

			else if(!strcmp(arg_name, "ssd_show_template")){
				ssd_show_template = atoi(arg_val);
			} else if(!strcmp(arg_name, "ssd_forgetting_factor")){
				ssd_forgetting_factor = atof(arg_val);
			}

			else if(!strcmp(arg_name, "buffer_count")){
				buffer_count = atoi(arg_val);
			} else if(!strcmp(arg_name, "nssd_norm_pix_max")){
				nssd_norm_pix_max = atof(arg_val);
			} else if(!strcmp(arg_name, "nssd_norm_pix_min")){
				nssd_norm_pix_min = atof(arg_val);
			} else if(!strcmp(arg_name, "zncc_likelihood_alpha")){
				zncc_likelihood_alpha = atof(arg_val);
			} else if(!strcmp(arg_name, "init_frame_id")){
				init_frame_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "start_frame_id")){
				start_frame_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "frame_gap")){
				frame_gap = atoi(arg_val);
			} else if(!strcmp(arg_name, "end_frame_id")){
				end_frame_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "show_tracking_error")){
				show_tracking_error = atoi(arg_val);
			} else if(!strcmp(arg_name, "write_tracking_error")){
				write_tracking_error = atoi(arg_val);
			} else if(!strcmp(arg_name, "tracking_err_type")){
				tracking_err_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "show_jaccard_error")){
				show_jaccard_error = atoi(arg_val);
			} else if(!strcmp(arg_name, "grad_eps")){
				grad_eps = strtod(arg_val, nullptr);
			} else if(!strcmp(arg_name, "hess_eps")){
				hess_eps = strtod(arg_val, nullptr);
			} else if(!strcmp(arg_name, "likelihood_alpha")){
				likelihood_alpha = strtod(arg_val, nullptr);
			} else if(!strcmp(arg_name, "likelihood_beta")){
				likelihood_beta = strtod(arg_val, nullptr);
			} else if(!strcmp(arg_name, "dist_from_likelihood")){
				dist_from_likelihood = atoi(arg_val);
			}
			//! Reinitialization
			else if(!strcmp(arg_name, "reinit_at_each_frame")){
				reinit_at_each_frame = atoi(arg_val);
			} else if(!strcmp(arg_name, "reset_at_each_frame")){
				reset_at_each_frame = atoi(arg_val);
			} else if(!strcmp(arg_name, "reset_to_init")){
				reset_to_init = atoi(arg_val);
			} else if(!strcmp(arg_name, "reinit_on_failure")){
				reinit_on_failure = atoi(arg_val);
			} else if(!strcmp(arg_name, "reinit_err_thresh")){
				reinit_err_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "reinit_frame_skip")){
				reinit_frame_skip = atoi(arg_val);
			} else if(!strcmp(arg_name, "reinit_with_new_obj")){
				reinit_with_new_obj = atoi(arg_val);
			}
			//! Reinitialization GT
			else if(!strcmp(arg_name, "use_reinit_gt")){
				use_reinit_gt = atoi(arg_val);
			}
			//! Optimized low DOF ground truth
			else if(!strcmp(arg_name, "use_opt_gt")){
				use_opt_gt = atoi(arg_val);
			} else if(!strcmp(arg_name, "opt_gt_ssm")){
				opt_gt_ssm = std::string(arg_val);
			}
			//! Pre processing type
			else if(!strcmp(arg_name, "pre_proc_type")){
				pre_proc_type = std::string(arg_val);
			} else if(!strcmp(arg_name, "pre_proc_hist_eq")){
				pre_proc_hist_eq = atoi(arg_val);
			}
			//! Gaussian smoothing
			else if(!strcmp(arg_name, "gauss_kernel_size")){
				gauss_kernel_size = atoi(arg_val);
			} else if(!strcmp(arg_name, "gauss_sigma_x")){
				gauss_sigma_x = atof(arg_val);
			} else if(!strcmp(arg_name, "gauss_sigma_y")){
				gauss_sigma_y = atof(arg_val);
			}
			//! Median Blurring
			else if(!strcmp(arg_name, "med_kernel_size")){
				med_kernel_size = atoi(arg_val);
			}
			//! Blurring with normalized box filter
			else if(!strcmp(arg_name, "box_kernel_size")){
				box_kernel_size = atoi(arg_val);
			}
			//! Bilateral Filtering
			else if(!strcmp(arg_name, "bil_diameter")){
				bil_diameter = atoi(arg_val);
			} else if(!strcmp(arg_name, "bil_sigma_col")){
				bil_sigma_col = atof(arg_val);
			} else if(!strcmp(arg_name, "bil_sigma_space")){
				bil_sigma_space = atof(arg_val);
			}
			//! Affine
			else if(!strcmp(arg_name, "aff_normalized_init")){
				aff_normalized_init = atoi(arg_val);
			} else if(!strcmp(arg_name, "aff_pt_based_sampling")){
				aff_pt_based_sampling = atoi(arg_val);
			}
			//! Homography
			else if(!strcmp(arg_name, "hom_normalized_init")){
				hom_normalized_init = atoi(arg_val);
			} else if(!strcmp(arg_name, "hom_corner_based_sampling")){
				hom_corner_based_sampling = atoi(arg_val);
			}
			//! Lie Homography
			else if(!strcmp(arg_name, "lhom_normalized_init")){
				lhom_normalized_init = atoi(arg_val);
			}else if(!strcmp(arg_name, "lhom_grad_eps")){
				lhom_grad_eps = atof(arg_val);
			}
			//! Similitude
			else if(!strcmp(arg_name, "sim_normalized_init")){
				sim_normalized_init = atoi(arg_val);
			} else if(!strcmp(arg_name, "sim_n_model_pts")){
				sim_n_model_pts = atoi(arg_val);
			} else if(!strcmp(arg_name, "sim_geom_sampling")){
				sim_geom_sampling = atoi(arg_val);
			} else if(!strcmp(arg_name, "sim_pt_based_sampling")){
				sim_pt_based_sampling = atoi(arg_val);
			}			
			//! Isometry
			else if(!strcmp(arg_name, "iso_pt_based_sampling")){
				iso_pt_based_sampling = atoi(arg_val);
			}
			//! SL3
			else if(!strcmp(arg_name, "sl3_normalized_init")){
				sl3_normalized_init = atoi(arg_val);
			} else if(!strcmp(arg_name, "sl3_iterative_sample_mean")){
				sl3_iterative_sample_mean = atoi(arg_val);
			} else if(!strcmp(arg_name, "sl3_sample_mean_max_iters")){
				sl3_sample_mean_max_iters = atoi(arg_val);
			} else if(!strcmp(arg_name, "sl3_sample_mean_eps")){
				sl3_sample_mean_eps = atof(arg_val);
			} else if(!strcmp(arg_name, "sl3_debug_mode")){
				sl3_debug_mode = atoi(arg_val);
			}
			//! Corner based Homography
			else if(!strcmp(arg_name, "chom_normalized_init")){
				chom_normalized_init = atoi(arg_val);
			} else if(!strcmp(arg_name, "chom_grad_eps")){
				chom_grad_eps = atof(arg_val);
			}
			//! Spline SSM
			else if(!strcmp(arg_name, "spl_control_size")){
				spl_control_size = atoi(arg_val);
			} else if(!strcmp(arg_name, "spl_control_overlap")){
				spl_control_overlap = atof(arg_val);
			} else if(!strcmp(arg_name, "spl_interp_type")){
				spl_interp_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "spl_static_wts")){
				spl_static_wts = atoi(arg_val);
			} else if(!strcmp(arg_name, "spl_debug_mode")){
				spl_debug_mode = atoi(arg_val);
			}
			//! SCV
			else if(!strcmp(arg_name, "scv_hist_type")){
				scv_hist_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "scv_use_bspl")){
				scv_use_bspl = atoi(arg_val);
			} else if(!strcmp(arg_name, "scv_n_bins")){
				scv_n_bins = atoi(arg_val);
			} else if(!strcmp(arg_name, "scv_preseed")){
				scv_preseed = atof(arg_val);
			} else if(!strcmp(arg_name, "scv_pou")){
				scv_pou = atoi(arg_val);
			} else if(!strcmp(arg_name, "scv_weighted_mapping")){
				scv_weighted_mapping = atoi(arg_val);
			} else if(!strcmp(arg_name, "scv_mapped_gradient")){
				scv_mapped_gradient = atoi(arg_val);
			} else if(!strcmp(arg_name, "scv_affine_mapping")){
				scv_affine_mapping = atoi(arg_val);
			} else if(!strcmp(arg_name, "scv_once_per_frame")){
				scv_once_per_frame = atoi(arg_val);
			} else if(!strcmp(arg_name, "scv_approx_dist_feat")){
				scv_approx_dist_feat = atoi(arg_val);
			} else if(!strcmp(arg_name, "scv_likelihood_alpha")){
				scv_likelihood_alpha = atof(arg_val);
			}
			//! LSCV
			else if(!strcmp(arg_name, "lscv_sub_regions")){
				lscv_sub_regions = atoi(arg_val);
			} else if(!strcmp(arg_name, "lscv_spacing")){
				lscv_spacing = atoi(arg_val);
			} else if(!strcmp(arg_name, "lscv_show_subregions")){
				lscv_show_subregions = atoi(arg_val);
			}
			//! LKLD
			else if(!strcmp(arg_name, "lkld_pre_seed")){
				lkld_pre_seed = atof(arg_val);
			} else if(!strcmp(arg_name, "lkld_pou")){
				lkld_pou = atoi(arg_val);
			} else if(!strcmp(arg_name, "lkld_n_bins")){
				lkld_n_bins = atoi(arg_val);
			} else if(!strcmp(arg_name, "lkld_sub_regions")){
				lkld_sub_regions = atoi(arg_val);
			} else if(!strcmp(arg_name, "lkld_spacing")){
				lkld_spacing = atoi(arg_val);
			}
			//! NCC
			else if(!strcmp(arg_name, "ncc_fast_hess")){
				ncc_fast_hess = atoi(arg_val);
			} else if(!strcmp(arg_name, "ncc_likelihood_alpha")){
				ncc_likelihood_alpha = atof(arg_val);
			}
			//! SPSS
			else if(!strcmp(arg_name, "spss_k")){
				spss_k = atof(arg_val);
			} else if(!strcmp(arg_name, "spss_likelihood_alpha")){
				spss_likelihood_alpha = atof(arg_val);
			}
			//! SSIM
			else if(!strcmp(arg_name, "ssim_pix_proc_type")){
				ssim_pix_proc_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "ssim_k1")){
				ssim_k1 = atof(arg_val);
			} else if(!strcmp(arg_name, "ssim_k2")){
				ssim_k2 = atof(arg_val);
			} else if(!strcmp(arg_name, "ssim_likelihood_alpha")){
				ssim_likelihood_alpha = atof(arg_val);
			}
			//! Sum of AMs
			else if(!strcmp(arg_name, "sum_am1")){
				sum_am1 = std::string(arg_val);
			} else if(!strcmp(arg_name, "sum_am2")){
				sum_am2 = std::string(arg_val);
			}
			//! Hessian
			else if(!strcmp(arg_name, "sec_ord_hess")){
				sec_ord_hess = atoi(arg_val);
			} else if(!strcmp(arg_name, "leven_marq")){
				leven_marq = atoi(arg_val);
			} else if(!strcmp(arg_name, "lm_delta_init")){
				lm_delta_init = atof(arg_val);
			} else if(!strcmp(arg_name, "lm_delta_update")){
				lm_delta_update = atof(arg_val);
			}
			//! Online learning in AM
			else if(!strcmp(arg_name, "enable_learning")){
				enable_learning = atof(arg_val);
			}
			//! ESM
			else if(!strcmp(arg_name, "esm_hess_type")){
				esm_hess_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "esm_jac_type")){
				esm_jac_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "esm_chained_warp")){
				esm_chained_warp = atoi(arg_val);
			}
			//! MI and CCRE
			else if(!strcmp(arg_name, "mi_pre_seed")){
				mi_pre_seed = atof(arg_val);
			} else if(!strcmp(arg_name, "mi_pou")){
				mi_pou = atoi(arg_val);
			} else if(!strcmp(arg_name, "mi_n_bins")){
				mi_n_bins = atoi(arg_val);
			} else if(!strcmp(arg_name, "mi_likelihood_alpha")){
				mi_likelihood_alpha = atof(arg_val);
			}
			//! CCRE
			else if(!strcmp(arg_name, "ccre_pre_seed")){
				ccre_pre_seed = atof(arg_val);
			} else if(!strcmp(arg_name, "ccre_pou")){
				ccre_pou = atoi(arg_val);
			} else if(!strcmp(arg_name, "ccre_n_bins")){
				ccre_n_bins = atoi(arg_val);
			} else if(!strcmp(arg_name, "ccre_symmetrical_grad")){
				ccre_symmetrical_grad = atoi(arg_val);
			} else if(!strcmp(arg_name, "ccre_n_blocks")){
				ccre_n_blocks = atoi(arg_val);
			} else if(!strcmp(arg_name, "ccre_likelihood_alpha")){
				ccre_likelihood_alpha = atof(arg_val);
			}
			//! NGF
			else if(!strcmp(arg_name, "ngf_eta")){
				ngf_eta = atof(arg_val);
			} else if(!strcmp(arg_name, "ngf_use_ssd")){
				ngf_use_ssd = atoi(arg_val);
			}
			//! diagnostics
			else if(!strcmp(arg_name, "diag_am")){
				processStringParam(diag_am, arg_val);
			} else if(!strcmp(arg_name, "diag_ssm")){
				processStringParam(diag_ssm, arg_val);
			} else if(!strcmp(arg_name, "diag_ilm")){
				processStringParam(diag_ilm, arg_val);
			} else if(!strcmp(arg_name, "diag_range")){
				diag_range = atof(arg_val);
			} else if(!strcmp(arg_name, "diag_ssm_range")){
				diag_ssm_range = atof_arr(arg_val);
			} else if(!strcmp(arg_name, "diag_ssm_range_id")){
				diag_ssm_range_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_am_range_id")){
				diag_am_range_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_res")){
				diag_res = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_3d")){
				diag_3d = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_3d_ids")){
				diag_3d_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "diag_update_type")){
				diag_update_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_start_id")){
				diag_start_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_end_id")){
				diag_end_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_grad_diff")){
				diag_grad_diff = atof(arg_val);
			} else if(!strcmp(arg_name, "diag_gen_norm")){
				diag_gen_norm = std::string(arg_val);
			} else if(!strcmp(arg_name, "diag_gen_jac")){
				diag_gen_jac = std::string(arg_val);
			} else if(!strcmp(arg_name, "diag_gen_hess")){
				diag_gen_hess = std::string(arg_val);
			} else if(!strcmp(arg_name, "diag_gen_hess2")){
				diag_gen_hess2 = std::string(arg_val);
			} else if(!strcmp(arg_name, "diag_gen_hess_sum")){
				diag_gen_hess_sum = std::string(arg_val);
			} else if(!strcmp(arg_name, "diag_gen_num")){
				diag_gen_num = std::string(arg_val);
			} else if(!strcmp(arg_name, "diag_gen_ssm")){
				diag_gen_ssm = std::string(arg_val);
			} else if(!strcmp(arg_name, "diag_bin")){
				diag_bin = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_inv")){
				diag_inv = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_frame_gap")){
				diag_frame_gap = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_show_data")){
				diag_show_data = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_show_corners")){
				diag_show_corners = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_show_patches")){
				diag_show_patches = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_verbose")){
				diag_verbose = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_enable_validation")){
				diag_enable_validation = atoi(arg_val);
			} else if(!strcmp(arg_name, "diag_validation_prec")){
				diag_validation_prec = atof(arg_val);
			} else if(!strcmp(arg_name, "diag_out_prefix")){
				diag_out_prefix = std::string(arg_val);
			}
			//! Selective Pixel Integration
			else if(!strcmp(arg_name, "esm_spi_enable")){
				esm_spi_enable = atoi(arg_val);
			} else if(!strcmp(arg_name, "esm_spi_thresh")){
				esm_spi_thresh = atof(arg_val);
			}

			else if(!strcmp(arg_name, "pix_mapper")){
				processStringParam(pix_mapper, arg_val);;
			}
			//! NN and GNN
			else if(!strcmp(arg_name, "nn_max_iters")){
				nn_max_iters = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_n_samples")){
				nn_n_samples = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_ssm_sigma_prec")){
				nn_ssm_sigma_prec = atof(arg_val);
			} else if(!strcmp(arg_name, "nn_additive_update")){
				nn_additive_update = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_show_samples")){
				nn_show_samples = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_add_points")){
				nn_add_points = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_remove_points")){
				nn_remove_points = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_ssm_sigma_ids")){
				nn_ssm_sigma_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "nn_ssm_mean_ids")){
				nn_ssm_mean_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "nn_corner_sigma_d")){
				nn_corner_sigma_d = atof(arg_val);
			} else if(!strcmp(arg_name, "nn_corner_sigma_t")){
				nn_corner_sigma_t = atof(arg_val);
			} else if(!strcmp(arg_name, "nn_pix_sigma")){
				nn_pix_sigma = atof_arr(arg_val);
			} else if(!strcmp(arg_name, "nn_n_trees")){
				nn_n_trees = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_n_checks")){
				nn_n_checks = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_index_type")){
				nn_index_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_search_type")){
				nn_search_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_save_index")){
				nn_save_index = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_load_index")){
				nn_load_index = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_saved_index_fid")){
				nn_saved_index_fid = atoi(arg_val);
			}
			//! NN Index specific parameters
			else if(!strcmp(arg_name, "nn_srch_checks")){
				nn_srch_checks = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_srch_eps")){
				nn_srch_eps = atof(arg_val);
			} else if(!strcmp(arg_name, "nn_srch_sorted")){
				nn_srch_sorted = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_srch_max_neighbors")){
				nn_srch_max_neighbors = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_srch_cores")){
				nn_srch_cores = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_srch_matrices_in_gpu_ram")){
				nn_srch_matrices_in_gpu_ram = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_srch_use_heap")){
				nn_srch_use_heap = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_gnn_degree")){
				nn_gnn_degree = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_gnn_max_steps")){
				nn_gnn_max_steps = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_gnn_cmpt_dist_thresh")){
				nn_gnn_cmpt_dist_thresh = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_gnn_random_start")){
				nn_gnn_random_start = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_fgnn_index_type")){
				nn_fgnn_index_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_gnn_verbose")){
				nn_gnn_verbose = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_kdt_trees")){
				nn_kdt_trees = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_km_branching")){
				nn_km_branching = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_km_iterations")){
				nn_km_iterations = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_km_centers_init")){
				nn_km_centers_init = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_km_cb_index")){
				nn_km_cb_index = atof(arg_val);
			} else if(!strcmp(arg_name, "nn_kdts_leaf_max_size")){
				nn_kdts_leaf_max_size = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_kdtc_leaf_max_size")){
				nn_kdtc_leaf_max_size = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_hc_branching")){
				nn_hc_branching = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_hc_centers_init")){
				nn_hc_centers_init = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_hc_trees")){
				nn_hc_trees = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_hc_leaf_max_size")){
				nn_hc_leaf_max_size = atoi(arg_val);
			} else if(!strcmp(arg_name, "nn_auto_target_precision")){
				nn_auto_target_precision = atof(arg_val);
			} else if(!strcmp(arg_name, "ann_uto_build_weight")){
				nn_auto_build_weight = atof(arg_val);
			} else if(!strcmp(arg_name, "nn_auto_memory_weight")){
				nn_auto_memory_weight = atof(arg_val);
			} else if(!strcmp(arg_name, "nn_auto_sample_fraction")){
				nn_auto_sample_fraction = atof(arg_val);
			} else if(!strcmp(arg_name, "nnk_n_layers")){
				nnk_n_layers = atoi(arg_val);
			} else if(!strcmp(arg_name, "nnk_ssm_sigma_ids")){
				nnk_ssm_sigma_ids.push_back(atoi_arr(arg_val));
			}
			//! Regression Network
			else if(!strcmp(arg_name, "rg_max_iters")){
				rg_max_iters = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_n_samples")){
				rg_n_samples = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_additive_update")){
				rg_additive_update = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_show_samples")){
				rg_show_samples = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_add_points")){
				rg_add_points = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_remove_points")){
				rg_remove_points = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_ssm_sigma_ids")){
				rg_ssm_sigma_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "rg_ssm_mean_ids")){
				rg_ssm_mean_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "rg_pix_sigma")){
				rg_pix_sigma = atof_arr(arg_val);
			} else if(!strcmp(arg_name, "rg_save_index")){
				rg_save_index = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_load_index")){
				rg_load_index = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_saved_index_fid")){
				rg_saved_index_fid = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_nepochs")){
				rg_nepochs = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_bs")){
				rg_bs = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_preproc")){
				rg_preproc = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_solver")){
				processStringParam(rg_solver, arg_val);;
			} else if(!strcmp(arg_name, "rg_train")){
				processStringParam(rg_train, arg_val);;
			} else if(!strcmp(arg_name, "rg_mean")){
				processStringParam(rg_mean, arg_val);;
			} else if(!strcmp(arg_name, "rg_dbg")){
				rg_dbg = atoi(arg_val);
			} else if(!strcmp(arg_name, "rg_pretrained")){
				rg_pretrained = atoi(arg_val);
			}

			//! RIU AM
			else if(!strcmp(arg_name, "riu_likelihood_alpha")){
				riu_likelihood_alpha = atof(arg_val);
			}
			//! Hierarchical SSM tracker
			else if(!strcmp(arg_name, "hrch_sm")){
				processStringParam(hrch_sm, arg_val);
			} else if(!strcmp(arg_name, "hrch_am")){
				processStringParam(hrch_am, arg_val);
			}
			//! Cascade Tracker
			if(!strcmp(arg_name, "casc_n_trackers")){
				casc_n_trackers = atoi(arg_val);
			} else 	if(!strcmp(arg_name, "casc_enable_feedback")){
				casc_enable_feedback = atoi(arg_val);
			} else if(!strcmp(arg_name, "casc_auto_reinit")) {
				casc_auto_reinit = atoi(arg_val);
			} else if(!strcmp(arg_name, "casc_reinit_err_thresh")) {
				casc_reinit_err_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "casc_reinit_frame_gap")) {
				casc_reinit_frame_gap = atof(arg_val);
			}
			//! Grid tracker
			else if(!strcmp(arg_name, "grid_sm")){
				processStringParam(grid_sm, arg_val);
			} else if(!strcmp(arg_name, "grid_am")){
				processStringParam(grid_am, arg_val);
			} else if(!strcmp(arg_name, "grid_ssm")){
				processStringParam(grid_ssm, arg_val);
			} else if(!strcmp(arg_name, "grid_ilm")){
				processStringParam(grid_ilm, arg_val);
			} else if(!strcmp(arg_name, "grid_res")){
				grid_res = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_patch_size")){
				grid_patch_size = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_patch_res")){
				grid_patch_res = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_reset_at_each_frame")){
				grid_reset_at_each_frame = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_dyn_patch_size")){
				grid_dyn_patch_size = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_patch_centroid_inside")){
				grid_patch_centroid_inside = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_show_trackers")){
				grid_show_trackers = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_show_tracker_edges")){
				grid_show_tracker_edges = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_use_tbb")){
				grid_use_tbb = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_pyramid_levels")){
				grid_pyramid_levels = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_use_min_eig_vals")){
				grid_use_min_eig_vals = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_min_eig_thresh")){
				grid_min_eig_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "grid_detect_keypoints")){
				grid_detect_keypoints = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_rebuild_index")){
				grid_rebuild_index = atoi(arg_val);
			} else if(!strcmp(arg_name, "grid_use_const_grad")){
				grid_use_const_grad = atoi(arg_val);
			}

			//! SIFT Feature Detector
			else if(!strcmp(arg_name, "sift_n_features")){
				sift_n_features = atoi(arg_val);
			} else if(!strcmp(arg_name, "sift_n_octave_layers")){
				sift_n_octave_layers = atoi(arg_val);
			} else if(!strcmp(arg_name, "sift_edge_thresh")){
				sift_edge_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "sift_contrast_thresh")){
				sift_contrast_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "sift_sigma")){
				sift_sigma = atof(arg_val);
			}
			//! SSM Estimator
			else if(!strcmp(arg_name, "est_method")){
				est_method = atoi(arg_val);
			} else if(!strcmp(arg_name, "est_ransac_reproj_thresh")){
				est_ransac_reproj_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "est_n_model_pts")){
				est_n_model_pts = atoi(arg_val);
			} else if(!strcmp(arg_name, "est_max_iters")){
				est_max_iters = atoi(arg_val);
			} else if(!strcmp(arg_name, "est_max_subset_attempts")){
				est_max_subset_attempts = atoi(arg_val);
			} else if(!strcmp(arg_name, "est_use_boost_rng")){
				est_use_boost_rng = atoi(arg_val);
			} else if(!strcmp(arg_name, "est_confidence")){
				est_confidence = atof(arg_val);
			} else if(!strcmp(arg_name, "est_refine")){
				est_refine = atoi(arg_val);
			} else if(!strcmp(arg_name, "est_lm_max_iters")){
				est_lm_max_iters = atoi(arg_val);
			}
			//! RKLT
			else if(!strcmp(arg_name, "rkl_sm")){
				processStringParam(rkl_sm, arg_val);
			} else if(!strcmp(arg_name, "rkl_enable_spi")){
				rkl_enable_spi = atoi(arg_val);
			} else if(!strcmp(arg_name, "rkl_enable_feedback")){
				rkl_enable_feedback = atoi(arg_val);
			} else if(!strcmp(arg_name, "rkl_failure_detection")){
				rkl_failure_detection = atoi(arg_val);
			} else if(!strcmp(arg_name, "rkl_failure_thresh")){
				rkl_failure_thresh = atof(arg_val);
			}
			//! Parallel Tracker
			else if(!strcmp(arg_name, "prl_n_trackers")) {
				prl_n_trackers = atoi(arg_val);
			} else if(!strcmp(arg_name, "prl_estimation_method")) {
				prl_estimation_method = atoi(arg_val);
			} else if(!strcmp(arg_name, "prl_reset_to_mean")) {
				prl_reset_to_mean = atoi(arg_val);
			} else if(!strcmp(arg_name, "prl_auto_reinit")) {
				prl_auto_reinit = atoi(arg_val);
			} else if(!strcmp(arg_name, "prl_reinit_err_thresh")) {
				prl_reinit_err_thresh = atof(arg_val);
			} else if(!strcmp(arg_name, "prl_reinit_frame_gap")) {
				prl_reinit_frame_gap = atof(arg_val);
			}
			//! Pyramidal Tracker
			else if(!strcmp(arg_name, "pyr_sm")) {
				pyr_sm = std::string(arg_val);
			} else if(!strcmp(arg_name, "pyr_no_of_levels")) {
				pyr_no_of_levels = atoi(arg_val);
			} else if(!strcmp(arg_name, "pyr_scale_factor")) {
				pyr_scale_factor = atof(arg_val);
			} else if(!strcmp(arg_name, "pyr_scale_res")) {
				pyr_scale_res = atoi(arg_val);
			}else if(!strcmp(arg_name, "pyr_show_levels")) {
				pyr_show_levels = atoi(arg_val);
			}
			//! Gradient Descent
			else if(!strcmp(arg_name, "gd_learning_rate")){
				gd_learning_rate = atof(arg_val);
			}
			//! Gain and Bias Illumination Model
			else if(!strcmp(arg_name, "gb_additive_update")){
				gb_additive_update = atoi(arg_val);
			}
			//! Piecewise Gain and Bias Illumination Model
			else if(!strcmp(arg_name, "pgb_additive_update")){
				pgb_additive_update = atoi(arg_val);
			} else if(!strcmp(arg_name, "pgb_sub_regions_x")){
				pgb_sub_regions_x = atoi(arg_val);
			} else if(!strcmp(arg_name, "pgb_sub_regions_y")){
				pgb_sub_regions_y = atoi(arg_val);
			}
			//! Radial Basis Function illumination model
			else if(!strcmp(arg_name, "rbf_additive_update")){
				rbf_additive_update = atoi(arg_val);
			} else if(!strcmp(arg_name, "rbf_n_ctrl_pts_x")){
				rbf_n_ctrl_pts_x = atoi(arg_val);
			} else if(!strcmp(arg_name, "rbf_n_ctrl_pts_y")){
				rbf_n_ctrl_pts_y = atoi(arg_val);
			}
			//! Particle Filter
			else if(!strcmp(arg_name, "pf_max_iters")){
				pf_max_iters = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_n_particles")){
				pf_n_particles = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_dynamic_model")){
				pf_dynamic_model = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_update_type")){
				pf_update_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_likelihood_func")){
				pf_likelihood_func = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_resampling_type")){
				pf_resampling_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_reset_to_mean")){
				pf_reset_to_mean = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_mean_type")){
				pf_mean_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_ssm_sigma_ids")){
				pf_ssm_sigma_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "pf_ssm_mean_ids")){
				pf_ssm_mean_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "pf_update_distr_wts")){
				pf_update_distr_wts = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_min_distr_wt")){
				pf_min_distr_wt = atof(arg_val);
			} else if(!strcmp(arg_name, "pf_measurement_sigma")){
				pf_measurement_sigma = atof(arg_val);
			} else if(!strcmp(arg_name, "pf_pix_sigma")){
				pf_pix_sigma = atof_arr(arg_val);
			} else if(!strcmp(arg_name, "pf_show_particles")){
				pf_show_particles = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_jacobian_as_sigma")){
				pf_jacobian_as_sigma = atoi(arg_val);
			} else if(!strcmp(arg_name, "pf_debug_mode")){
				pf_debug_mode = atoi(arg_val);
			}
			//! Multi layer PF
			else if(!strcmp(arg_name, "pfk_n_layers")){
				pfk_n_layers = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfk_ssm_sigma_ids")){
				pfk_ssm_sigma_ids.push_back(atoi_arr(arg_val));
			}
			//! Gaussian parameters for sampling SSM parameters
			else if(!strcmp(arg_name, "ssm_sigma")){
				ssm_sigma.push_back(atof_arr(arg_val));
			} else if(!strcmp(arg_name, "ssm_mean")){
				ssm_mean.push_back(atof_arr(arg_val));
			} else if(!strcmp(arg_name, "am_sigma")){
				am_sigma.push_back(atof_arr(arg_val));
			} else if(!strcmp(arg_name, "am_mean")){
				am_mean.push_back(atof_arr(arg_val));
			}
			//! CMT
			else if(!strcmp(arg_name, "cmt_estimate_scale")){
				cmt_estimate_scale = atoi(arg_val);
			} else if(!strcmp(arg_name, "cmt_estimate_rotation")){
				cmt_estimate_rotation = atoi(arg_val);
			} else if(!strcmp(arg_name, "cmt_feat_detector")){
				processStringParam(cmt_feat_detector, arg_val);
			} else if(!strcmp(arg_name, "cmt_desc_extractor")){
				processStringParam(cmt_desc_extractor, arg_val);
			} else if(!strcmp(arg_name, "cmt_resize_factor")){
				cmt_resize_factor = atof(arg_val);
			}
			//! DSST
			else if(!strcmp(arg_name, "dsst_sigma")){
				dsst_sigma = atof(arg_val);
			} else if(!strcmp(arg_name, "dsst_scale_sigma")){
				dsst_scale_sigma = atof(arg_val);
			} else if(!strcmp(arg_name, "dsst_lambda")){
				dsst_lambda = atof(arg_val);
			} else if(!strcmp(arg_name, "dsst_learning_rate")){
				dsst_learning_rate = atof(arg_val);
			} else if(!strcmp(arg_name, "dsst_number_scales")){
				dsst_number_scales = atoi(arg_val);
			} else if(!strcmp(arg_name, "dsst_number_rots")){
				dsst_number_rots = atoi(arg_val);
			} else if(!strcmp(arg_name, "dsst_scale_step")){
				dsst_scale_step = atof(arg_val);
			} else if(!strcmp(arg_name, "dsst_rot_step")){
				dsst_rot_step = atof(arg_val);
			} else if(!strcmp(arg_name, "dsst_padding")){
				dsst_padding = atoi(arg_val);
			} else if(!strcmp(arg_name, "dsst_resize_factor")){
				dsst_resize_factor = atoi(arg_val);
			} else if(!strcmp(arg_name, "dsst_is_scaling")){
				dsst_is_scaling = atoi(arg_val);
			} else if(!strcmp(arg_name, "dsst_is_rotating")){
				dsst_is_rotating = atoi(arg_val);
			} else if(!strcmp(arg_name, "dsst_bin_size")){
				dsst_bin_size = atoi(arg_val);
			}
			//! KCF
			else if(!strcmp(arg_name, "kcf_output_sigma_factor")){
				kcf_output_sigma_factor = atof(arg_val);
			} else if(!strcmp(arg_name, "kcf_interp_factor")){
				kcf_interp_factor = atof(arg_val);
			} else if(!strcmp(arg_name, "kcf_lambda")){
				kcf_lambda = atof(arg_val);
			} else if(!strcmp(arg_name, "kcf_kernel_sigma")){
				kcf_kernel_sigma = atof(arg_val);
			} else if(!strcmp(arg_name, "kcf_number_scales")){
				kcf_number_scales = atoi(arg_val);
			} else if(!strcmp(arg_name, "kcf_scale_step")){
				kcf_scale_step = atof(arg_val);
			} else if(!strcmp(arg_name, "kcf_padding")){
				kcf_padding = atoi(arg_val);
			} else if(!strcmp(arg_name, "kcf_resize_factor")){
				kcf_resize_factor = atoi(arg_val);
			} else if(!strcmp(arg_name, "kcf_scale_model_max_area")){
				kcf_scale_model_max_area = atof(arg_val);
			} else if(!strcmp(arg_name, "kcf_scale_sigma_factor")){
				kcf_scale_sigma_factor = atof(arg_val);
			} else if(!strcmp(arg_name, "kcf_scale_learning_rate")){
				kcf_scale_learning_rate = atof(arg_val);
			} else if(!strcmp(arg_name, "kcf_enableScaling")){
				kcf_enableScaling = atoi(arg_val);
			}
			//! MIL
			else if(!strcmp(arg_name, "mil_algorithm")){
				mil_algorithm = atoi(arg_val);
			} else if(!strcmp(arg_name, "mil_num_classifiers")){
				mil_num_classifiers = atoi(arg_val);
			} else if(!strcmp(arg_name, "mil_overlap")){
				mil_overlap = atof(arg_val);
			} else if(!strcmp(arg_name, "mil_search_factor")){
				mil_search_factor = atof(arg_val);
			} else if(!strcmp(arg_name, "mil_pos_radius_train")){
				mil_pos_radius_train = atof(arg_val);
			} else if(!strcmp(arg_name, "mil_neg_num_train")){
				mil_neg_num_train = atoi(arg_val);
			} else if(!strcmp(arg_name, "mil_num_features")){
				mil_num_features = atoi(arg_val);
			}
			//! TLD
			else if(!strcmp(arg_name, "tld_detector_enabled")){
				tld_detector_enabled = atoi(arg_val);
			} else if(!strcmp(arg_name, "tld_learning_enabled")){
				tld_learning_enabled = atoi(arg_val);
			} else if(!strcmp(arg_name, "tld_tracker_enabled")){
				tld_tracker_enabled = atoi(arg_val);
			} else if(!strcmp(arg_name, "tld_alternating")){
				tld_alternating = atoi(arg_val);
			}
			//! RCT
			else if(!strcmp(arg_name, "rct_min_n_rect")){
				rct_min_n_rect = atoi(arg_val);
			} else if(!strcmp(arg_name, "rct_max_n_rect")){
				rct_max_n_rect = atoi(arg_val);
			} else if(!strcmp(arg_name, "rct_n_feat")){
				rct_n_feat = atoi(arg_val);
			} else if(!strcmp(arg_name, "rct_rad_outer_pos")){
				rct_rad_outer_pos = atoi(arg_val);
			} else if(!strcmp(arg_name, "rct_rad_search_win")){
				rct_rad_search_win = atoi(arg_val);
			} else if(!strcmp(arg_name, "rct_learning_rate")){
				rct_learning_rate = atof(arg_val);
			}
			//! Struck
			else if(!strcmp(arg_name, "strk_config_path")){
				strk_config_path = std::string(arg_val);
			}
			//! ViSP
			else if(!strcmp(arg_name, "visp_sm")){
				processStringParam(visp_sm, arg_val);
			} else if(!strcmp(arg_name, "visp_am")){
				processStringParam(visp_am, arg_val);
			} else if(!strcmp(arg_name, "visp_ssm")){
				processStringParam(visp_ssm, arg_val);
			} else if(!strcmp(arg_name, "visp_max_iters")){
				visp_max_iters = atoi(arg_val);
			} else if(!strcmp(arg_name, "visp_res")){
				visp_res = atoi(arg_val);
			} else if(!strcmp(arg_name, "visp_pyr_n_levels")){
				visp_pyr_n_levels = atoi(arg_val);
			} else if(!strcmp(arg_name, "visp_pyr_level_to_stop")){
				visp_pyr_level_to_stop = atoi(arg_val);
			} else if(!strcmp(arg_name, "visp_lambda")){
				visp_lambda = atof(arg_val);
			} else if(!strcmp(arg_name, "visp_thresh_grad")){
				visp_thresh_grad = atof(arg_val);
			} else if(!strcmp(arg_name, "visp_fw_res")){
				visp_fw_res = atoi(arg_val);
			} else if(!strcmp(arg_name, "visp_fw_fps")){
				visp_fw_fps = atoi(arg_val);
			} else if(!strcmp(arg_name, "visp_usb_res")){
				visp_usb_res = atoi(arg_val);
			} else if(!strcmp(arg_name, "visp_usb_fps")){
				visp_usb_fps = atoi(arg_val);
			} else if(!strcmp(arg_name, "visp_usb_n_buffers")){
				visp_usb_n_buffers = atoi(arg_val);
			}
			//! PFSL3
			else if(!strcmp(arg_name, "pfsl3_p_x")){
				pfsl3_p_x = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_p_y")){
				pfsl3_p_y = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_rot")){
				pfsl3_rot = atof(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_ncc_std")){
				pfsl3_ncc_std = atof(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_pca_std")){
				pfsl3_pca_std = atof(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_state_std")){
				pfsl3_state_std = atof_arr(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_ar_p")){
				pfsl3_ar_p = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_n")){
				pfsl3_n = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_n_c")){
				pfsl3_n_c = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_n_iter")){
				pfsl3_n_iter = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_sampling")){
				pfsl3_sampling = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_capture")){
				pfsl3_capture = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_mean_check")){
				pfsl3_mean_check = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_outlier_flag")){
				pfsl3_outlier_flag = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_len")){
				pfsl3_len = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_init_size")){
				pfsl3_init_size = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_update_period")){
				pfsl3_update_period = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_ff")){
				pfsl3_ff = atof(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_basis_thr")){
				pfsl3_basis_thr = atof(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_max_num_basis")){
				pfsl3_max_num_basis = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_max_num_used_basis")){
				pfsl3_max_num_used_basis = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_show_weights")){
				pfsl3_show_weights = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_show_templates")){
				pfsl3_show_templates = atoi(arg_val);
			} else if(!strcmp(arg_name, "pfsl3_debug_mode")){
				pfsl3_debug_mode = atoi(arg_val);
			}

			//! GOTURN
			else if(!strcmp(arg_name, "gtrn_do_train")){
				gtrn_do_train = atoi(arg_val);
			} else if(!strcmp(arg_name, "gtrn_gpu_id")){
				gtrn_gpu_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "gtrn_show_intermediate_output")){
				gtrn_show_intermediate_output = atoi(arg_val);
			} else if(!strcmp(arg_name, "gtrn_model_file")){
				gtrn_model_file = std::string(arg_val);
			} else if(!strcmp(arg_name, "gtrn_trained_file")){
				gtrn_trained_file = std::string(arg_val);
			}
			//! DFT
			else if(!strcmp(arg_name, "dft_res_to_l")){
				dft_res_to_l = atof(arg_val);
			} else if(!strcmp(arg_name, "dft_p_to_l")){
				dft_p_to_l = atof(arg_val);
			} else if(!strcmp(arg_name, "dft_max_iter")){
				dft_max_iter = atoi(arg_val);
			} else if(!strcmp(arg_name, "dft_max_iter_single_level")){
				dft_max_iter_single_level = atoi(arg_val);
			} else if(!strcmp(arg_name, "dft_pyramid_smoothing_variance")){
				dft_pyramid_smoothing_variance = atof32_arr(arg_val);
			} else if(!strcmp(arg_name, "dft_presmoothing_variance")){
				dft_presmoothing_variance = atof(arg_val);
			} else if(!strcmp(arg_name, "dft_n_control_points_on_edge")){
				dft_n_control_points_on_edge = atoi(arg_val);
			} else if(!strcmp(arg_name, "dft_b_adaptative_choice_of_points")){
				dft_b_adaptative_choice_of_points = atoi(arg_val);
			} else if(!strcmp(arg_name, "dft_b_normalize_descriptors")){
				dft_b_normalize_descriptors = atoi(arg_val);
			} else if(!strcmp(arg_name, "dft_optimization_type")){
				dft_optimization_type = atoi(arg_val);
			}
			// FRG
			else if(!strcmp(arg_name, "frg_n_bins")){
				frg_n_bins = atoi(arg_val);
			} else if(!strcmp(arg_name, "frg_search_margin")){
				frg_search_margin = atoi(arg_val);
			} else if(!strcmp(arg_name, "frg_hist_cmp_metric")){
				frg_hist_cmp_metric = atoi(arg_val);
			} else if(!strcmp(arg_name, "frg_resize_factor")){
				frg_resize_factor = atof(arg_val);
			} else if(!strcmp(arg_name, "frg_show_window")){
				frg_show_window = atoi(arg_val);
			}
			//! FMaps
			else if(!strcmp(arg_name, "dfm_nfmaps")){
				dfm_nfmaps = atoi(arg_val);
			} else if(!strcmp(arg_name, "dfm_layer_name")){
				processStringParam(dfm_layer_name, arg_val);
			} else if(!strcmp(arg_name, "dfm_vis")){
				dfm_vis = atoi(arg_val);
			} else if(!strcmp(arg_name, "dfm_zncc")){
				dfm_zncc = atoi(arg_val);
			} else if(!strcmp(arg_name, "dfm_model_f_name")){
				processStringParam(dfm_model_f_name, arg_val);
			} else if(!strcmp(arg_name, "dfm_params_f_name")){
				processStringParam(dfm_params_f_name, arg_val);
			} else if(!strcmp(arg_name, "dfm_mean_f_name")){
				processStringParam(dfm_mean_f_name, arg_val);
			}
			//! Patch extractor
			else if(!strcmp(arg_name, "extracted_frame_ids")){
				extracted_frame_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "extraction_id")){
				extraction_id = atoi(arg_val);
			}
			//! PCA
			else if(!strcmp(arg_name, "pca_n_eigenvec")){
				pca_n_eigenvec = atoi(arg_val);
			} else if(!strcmp(arg_name, "pca_batchsize")){
				pca_batchsize = atoi(arg_val);
			} else if(!strcmp(arg_name, "pca_f_factor")){
				pca_f_factor = atof(arg_val);
			} else if(!strcmp(arg_name, "pca_show_basis")){
				pca_show_basis = atoi(arg_val);
			}

			//! Synthetic warped sequence generator
			else if(!strcmp(arg_name, "syn_ssm")){
				syn_ssm = std::string(arg_val);
			} else if(!strcmp(arg_name, "syn_ilm")){
				syn_ilm = std::string(arg_val);
			} else if(!strcmp(arg_name, "syn_frame_id")){
				syn_frame_id = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_grayscale_img")){
				syn_grayscale_img = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_continuous_warping")){
				syn_continuous_warping = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_ssm_sigma_ids")){
				syn_ssm_sigma_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "syn_ssm_mean_ids")){
				syn_ssm_mean_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "syn_am_sigma_ids")){
				syn_am_sigma_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "syn_am_mean_ids")){
				syn_am_mean_ids = atoi_arr(arg_val);
			} else if(!strcmp(arg_name, "syn_pix_sigma")){
				syn_pix_sigma = atof(arg_val);
			} else if(!strcmp(arg_name, "syn_am_on_obj")){
				syn_am_on_obj = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_warp_entire_image")){
				syn_warp_entire_image = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_background_type")){
				syn_background_type = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_use_inv_warp")){
				syn_use_inv_warp = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_out_suffix")){
				syn_out_suffix = std::string(arg_val);
			} else if(!strcmp(arg_name, "syn_n_frames")){
				syn_n_frames = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_add_noise")){
				syn_add_noise = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_noise_mean")){
				syn_noise_mean = atof(arg_val);
			} else if(!strcmp(arg_name, "syn_noise_sigma")){
				syn_noise_sigma = atof(arg_val);
			} else if(!strcmp(arg_name, "syn_save_as_video")){
				syn_save_as_video = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_video_fps")){
				syn_video_fps = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_jpg_quality")){
				syn_jpg_quality = atoi(arg_val);
			} else if(!strcmp(arg_name, "syn_show_output")){
				syn_show_output = atoi(arg_val);
			}

			else if(!strcmp(arg_name, "mos_track_border")){
				mos_track_border = atoi(arg_val);
			} else if(!strcmp(arg_name, "mos_border")){
				mos_border = atoi(arg_val);
			} else if(!strcmp(arg_name, "mos_disp_width")){
				mos_disp_width = atoi(arg_val);
			} else if(!strcmp(arg_name, "mos_disp_height")){
				mos_disp_height = atoi(arg_val);
			} else if(!strcmp(arg_name, "mos_show_grid")){
				mos_show_grid = atoi(arg_val);
			} else if(!strcmp(arg_name, "mos_use_write_mask")){
				mos_use_write_mask = atoi(arg_val);
			}
		}

		inline FILE* readTrackerParams(FILE * fid = nullptr, int print_args = 0){
			// reads parameters for a single tracker in a multi tracker setup
			// a blank line signals the end of the current tracker's parameters
			// a line starting with # is treated as comment and ignored
			if(!fid){
				const char *fname = (config_dir + "/multi.cfg").c_str();
				printf("Opening file: %s...\n", fname);
				if(!(fid = fopen(fname, "r"))){
					printf("readTrackerParams :: Error: File could not be opened\n");
					return nullptr;
				}
			}
			char curr_line[500];
			char arg_name[500], arg_val[500];
			while(!feof(fid)){
				fgets(curr_line, 500, fid);
				strtok(curr_line, "\n");
				strtok(curr_line, "\r");
				// ignore comments
				if(curr_line[0] == '#')
					continue;
				// empty line signals the end of current tracker's parameters
				if(strlen(curr_line) <= 1)
					return fid;
				sscanf(curr_line, "%s%s", arg_name, arg_val);
				strtok(arg_name, "\n");
				strtok(arg_name, "\r");
				strtok(arg_val, "\n");
				strtok(arg_val, "\r");
				char *alpha_arg_name = arg_name;
				while(!isalpha(*alpha_arg_name)){ ++alpha_arg_name; }
				if(strlen(alpha_arg_name) == 0){ continue; }
				if(print_args)
					printf("arg_name: %s arg_val: %s\n", alpha_arg_name, arg_val);
				processAgrument(alpha_arg_name, arg_val);
			}
			fclose(fid);
			return nullptr;
		}

		inline bool parseArgumentPairs(char * argv[], int argc,
			int parse_type = 0, int print_args = 0){
			// parse_type = 0 means that each element of argv contains
			// an argument's name and its value separated by a space
			// parse_type = 1 means that consecutive elements of argv contain
			// the name and value of each argument
			if(argc <= 0){ return true; }

			//printf("Parsing %d argument pairs with argv=%d...\n", argc, argv);			
			int max_arg = parse_type ? argc / 2 + 1 : argc;
			if(print_args){
				printf("argc: %d max_arg: %d\n", argc, max_arg);
			}
			for(int i = 1; i < max_arg; i++){
				char arg_name[500], arg_val[500];
				//printf("i=%d\n", i);
				if(parse_type){
					sscanf(argv[2 * i - 1], "%s", arg_name);
					sscanf(argv[2 * i], "%s", arg_val);
				} else{
					sscanf(argv[i], "%s%s", arg_name, arg_val);
				}
				strtok(arg_name, "\n");
				strtok(arg_name, "\r");
				strtok(arg_val, "\n");
				strtok(arg_val, "\r");
				// discard all numbers and special characters from the start of the argument name
				char *alpha_arg_name = arg_name;
				vector<char> arg_prefix;
				while(!isalpha(*alpha_arg_name)){
					arg_prefix.push_back(*alpha_arg_name);
					++alpha_arg_name;
				}
				if(strlen(alpha_arg_name) == 0){ continue; }
				if(print_args){
					printf("arg_name: %s arg_val: %s\n", alpha_arg_name, arg_val);
				}
				processAgrument(alpha_arg_name, arg_val, arg_prefix.data());
			}
			return true;
		}

		inline std::string getSyntheticSeqName(){
			if(syn_out_suffix.empty()){
				syn_out_suffix = cv::format("warped_%s_s%d", syn_ssm.c_str(), syn_ssm_sigma_ids[0]);
				if(syn_ilm != "0"){
					syn_out_suffix = cv::format("%s_%s_s%d", syn_out_suffix.c_str(),
						syn_ilm.c_str(), syn_am_sigma_ids[0]);
				}
				if(syn_add_noise){
					syn_out_suffix = cv::format("%s_gauss_%4.2f_%4.2f", syn_out_suffix.c_str(),
						syn_noise_mean, syn_noise_sigma);
				}
			}
			return cv::format("%s_%d_%s", source_name.c_str(), syn_frame_id, syn_out_suffix.c_str());
		}
		inline bool postProcessParams(){
			if(mtf_res > 0){ resx = resy = mtf_res; }
			if(img_resize_factor <= 0){ img_resize_factor = 1; }

			if((img_source == SRC_IMG) || (img_source == SRC_DISK) || (img_source == SRC_VID)){
				if(actor_id >= 0){
					int n_actors = sizeof(actors) / sizeof(actors[0]);
					//printf("n_actors: %d\n", n_actors);
					if(actor_id >= n_actors){
						printf("Invalid actor id specified: %d\n", actor_id);
						return false;
					}
					actor = actors[actor_id];

					if(source_id >= 0){
						int n_sources = combined_n_sources[actor_id];
						if(source_id >= n_sources){
							printf("Invalid source id %d specified for actor %s with %d sources\n",
								source_id, actor.c_str(), n_sources);
							return false;
						}
						source_name = combined_sources[actor_id][source_id];
					}
					if(actor == "Synthetic"){
						//! synthetic sequence
						source_name = getSyntheticSeqName();
					}
				}
				source_path = db_root_path + "/" + actor;
				if(source_fmt == "#"){
					source_fmt = (img_source == SRC_IMG || img_source == SRC_DISK) ? IMG_FMT : VID_FMT;
				}
			} else {
				actor = "Live";
				source_name = (img_source == SRC_USB_CAM) ? USB_DEV_NAME : FW_DEV_NAME;
				if(source_path == "#"){
					source_path = (img_source == SRC_USB_CAM) ? USB_DEV_PATH : FW_DEV_PATH;
				}
				if(source_fmt == "#"){
					source_fmt = (img_source == SRC_USB_CAM) ? USB_DEV_FMT : FW_DEV_FMT;
				}
				show_tracking_error = reinit_on_failure = read_obj_from_gt = read_obj_from_file = pause_after_frame = 0;
			}
			return true;
		}
		inline bool readParams(int cmd_argc, char* cmd_argv[]){
			//! check if a custom configuration directory has been specified
			if(cmd_argc > 2 && !strcmp(cmd_argv[1], "config_dir")){
				config_dir = std::string(cmd_argv[2]);
				printf("Reading configuration files from: %s\n", config_dir.c_str());
			}
			std::vector<char*> fargv;
			//! read general parameters
			int fargc = readParams(fargv, (config_dir + "/mtf.cfg").c_str());
			if(fargc){
				if(!parseArgumentPairs(fargv.data(), fargc)){
					printf("Error in parsing mtf.cfg\n");
					return false;
				}
				fargv.clear();
			}
			//! read parameters specific to different modules
			fargc = readParams(fargv, (config_dir + "/modules.cfg").c_str());
			if(fargc){
				if(!parseArgumentPairs(fargv.data(), fargc)){
					printf("Error in parsing modules.cfg\n");
					return false;
				}
				fargv.clear();
			}
			//! read standard deviations and means for stochastic modules
			fargc = readParams(fargv, (config_dir + "/sigma.cfg").c_str());
			if(fargc){
				if(!parseArgumentPairs(fargv.data(), fargc)){
					printf("Error in parsing sigma.cfg\n");
					return false;
				}
				fargv.clear();
			}
			//! read parameters for third party trackers
			fargc = readParams(fargv, (config_dir + "/thirdparty.cfg").c_str());
			if(fargc){
				if(!parseArgumentPairs(fargv.data(), fargc)){
					printf("Error in parsing thirdparty.cfg\n");
					return false;
				}
				fargv.clear();
			}
			//! parse command line arguments
			if(!parseArgumentPairs(cmd_argv, cmd_argc, 1, 0)){
				printf("Error in parsing command line arguments\n");
				return false;
			}
			if(!postProcessParams()){
				printf("Error in post processing params\n");
				return false;
			}
			return true;
		}
		inline void getSamplerParams(vectorvd &sigma, vectorvd &mean,
			const vectori &sigma_ids, const vectori &mean_ids,
			const char* name){
			//printf("ssm_sigma.size(): %ld\n", ssm_sigma.size());
			//for(int i = 0; i < ssm_sigma.size(); ++i){
			//	for(vectord::const_iterator iter = ssm_sigma[i].begin(); iter != ssm_sigma[i].end(); ++iter){
			//		printf("%f\t", *iter);			
			//	}
			//	printf("\n");
			//}
			//printf("ssm_mean.size(): %ld\n", ssm_mean.size());
			//for(int i = 0; i < ssm_mean.size(); ++i){
			//	for(vectord::const_iterator iter = ssm_mean[i].begin(); iter != ssm_mean[i].end(); ++iter){
			//		printf("%f\t", *iter);
			//	}
			//	printf("\n");
			//}
			for(int i = 0; i < sigma_ids.size(); ++i){
				int sigma_id = sigma_ids[i];
				if(sigma_id < 0 || sigma_id >= ssm_sigma.size()){
					printf("Skipping invalid %s sigma ID: %d\n", name, sigma_id);
					continue;
				}
				sigma.push_back(ssm_sigma[sigma_id]);
			}
			for(int i = 0; i < mean_ids.size(); ++i){
				int mean_id = mean_ids[i];
				if(mean_id < 0 || mean_id >= ssm_mean.size()){
					//printf("Skipping invalid %s SSM mean ID: %d\n", name, mean_id);
					continue;
				}
				mean.push_back(ssm_mean[mean_id]);
			}
		}
		inline void getAMSamplerParams(vectorvd &sigma, vectorvd &mean,
			const vectori &sigma_ids, const vectori &mean_ids,
			const char* name){
			for(int i = 0; i < sigma_ids.size(); ++i){
				int sigma_id = sigma_ids[i];
				if(sigma_id < 0 || sigma_id >= am_sigma.size()){
					printf("Skipping invalid %s sigma ID: %d\n", name, sigma_id);
					continue;
				}
				sigma.push_back(am_sigma[sigma_id]);
			}
			for(int i = 0; i < mean_ids.size(); ++i){
				int mean_id = mean_ids[i];
				if(mean_id < 0 || mean_id >= am_mean.size()){
					continue;
				}
				mean.push_back(am_mean[mean_id]);
			}
		}

		inline void freeParams(){}
	}
}



#endif
