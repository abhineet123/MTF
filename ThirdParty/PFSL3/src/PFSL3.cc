#include <stdlib.h>

extern "C" {
#include <f2c.h>
#include <clapack.h>
}
#include "mtf/ThirdParty/PFSL3/PFSL3.h"
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#define print_array(arr, arr_size, arr_name, format, fid)\
	fprintf(fid, arr_name);\
	fprintf(fid, ":\n");\
	for(int i = 0; i < arr_size; ++i){\
		fprintf(fid, format, arr[i]);\
		fprintf(fid, "\t");\
				}\
	fprintf(fid, "\n")

#define print_mat(mat, mat_name, data_type, format, fid)\
	fprintf(fid, mat_name);\
	fprintf(fid, ":\n");\
	for(int row_id = 0; row_id < mat->rows; ++row_id){\
		for(int col_id = 0; col_id < mat->cols; ++col_id){\
			fprintf(fid, format, CV_MAT_ELEM(*mat, data_type, row_id, col_id)); \
			fprintf(fid, "\t");\
				}\
		fprintf(fid, "\n");\
	}\
	fprintf(fid, "\n")

PFSL3Params::PFSL3Params(int _p_x, int _p_y,
	const std::vector<double> &_state_std,
	double _rot, double _NCC_std, double _PCA_std, double _AR_p,
	int _N, int _N_c, int _N_iter, int _sampling, int _capture,
	int _mean_check, int _outlier_flag, int _len,
	int _init_size, int _update_period, float _ff,
	double _basis_thr, int _max_num_basis, int _max_num_used_basis,
	bool _show_weights, bool _show_templates, bool _debug_mode):
	p_x(_p_x),
	p_y(_p_y),
	state_std(_state_std),
	rot(_rot),
	NCC_std(_NCC_std),
	PCA_std(_PCA_std),
	AR_p(_AR_p),
	N(_N),
	N_c(_N_c),
	N_iter(_N_iter),
	sampling(_sampling),
	capture(_capture),
	mean_check(_mean_check),
	outlier_flag(_outlier_flag),
	len(_len),
	init_size(_init_size),
	update_period(_update_period),
	ff(_ff),
	basis_thr(_basis_thr),
	max_num_basis(_max_num_basis),
	max_num_used_basis(_max_num_used_basis),
	show_weights(_show_weights),
	show_templates(_show_templates),
	debug_mode(_debug_mode){}

PFSL3Params::PFSL3Params(const PFSL3Params *params) :
p_x(PFSL3_P_X),
p_y(PFSL3_P_Y),
state_std({ PFSL3_STATE_STD }),
rot(PFSL3_ROT),
NCC_std(PFSL3_NCC_STD),
PCA_std(PFSL3_PCA_STD),
AR_p(PFSL3_AR_P),
N(PFSL3_N),
N_c(PFSL3_N_C),
N_iter(PFSL3_N_ITER),
sampling(PFSL3_SAMPLING),
capture(PFSL3_CAPTURE),
mean_check(PFSL3_MEAN_CHECK),
outlier_flag(PFSL3_OUTLIER_FLAG),
len(PFSL3_LEN),
init_size(PFSL3_INIT_SIZE),
update_period(PFSL3_UPDATE_PERIOD),
ff(PFSL3_FF),
basis_thr(PFSL3_BASIS_THR),
max_num_basis(PFSL3_MAX_NUM_BASIS),
max_num_used_basis(PFSL3_MAX_NUM_USED_BASIS),
show_weights(PFSL3_SHOW_WEIGHTS),
show_templates(PFSL3_SHOW_TEMPLATES),
debug_mode(PFSL3_DEBUG_MODE){
	if(params) {
		p_x = params->p_x;
		p_y = params->p_y;
		state_std = params->state_std;
		rot = params->rot;
		NCC_std = params->NCC_std;
		PCA_std = params->PCA_std;
		AR_p = params->AR_p;
		N = params->N;
		N_c = params->N_c;
		N_iter = params->N_iter;
		sampling = params->sampling;
		capture = params->capture;
		mean_check = params->mean_check;
		outlier_flag = params->outlier_flag;
		len = params->len;
		init_size = params->init_size;
		update_period = params->update_period;
		ff = params->ff;
		basis_thr = params->basis_thr;
		max_num_basis = params->max_num_basis;
		max_num_used_basis = params->max_num_used_basis;
		show_weights = params->show_weights;
		show_templates = params->show_templates;
		show_templates = params->show_templates;
		debug_mode = params->debug_mode;
	}
}
PFSL3::PFSL3(const ParamType *pfsl3_params) :
TrackerBase(), params(pfsl3_params) {
	printf("\n");
	printf("Using PFSL3 tracker with: \n");
	printf("p_x: %d\n", params.p_x);
	printf("p_y: %d\n", params.p_y);
	printf("state_std: \n");
	for(int state_id = 0; state_id < params.state_std.size(); ++state_id){
		printf("%f\t", params.state_std[state_id]);
	}
	printf("\n");
	printf("rot: %f\n", params.rot);
	printf("NCC_std: %f\n", params.NCC_std);
	printf("PCA_std: %f\n", params.PCA_std);
	printf("AR_p: %f\n", params.AR_p);
	printf("N: %d\n", params.N);
	printf("N_c: %d\n", params.N_c);
	printf("N_iter: %d\n", params.N_iter);
	printf("sampling: %d\n", params.sampling);
	printf("capture: %d\n", params.capture);
	printf("mean_check: %d\n", params.mean_check);
	printf("outlier_flag: %d\n", params.outlier_flag);
	printf("len: %d\n", params.len);
	printf("init_size: %d\n", params.init_size);
	printf("update_period: %d\n", params.update_period);
	printf("ff: %f\n", params.ff);
	printf("basis_thr: %f\n", params.basis_thr);
	printf("max_num_basis: %d\n", params.max_num_basis);
	printf("max_num_used_basis: %d\n", params.max_num_used_basis);
	printf("show_weights: %d\n", params.show_weights);
	printf("show_templates: %d\n", params.show_templates);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("\n");
	name = "pfsl3";
	cv_corners_mat.create(2, 4, CV_64FC1);
	debug_fid = fopen("log/pfsl3.txt", "w");
	if(params.debug_mode){
		printf("Writing debug output to log/pfsl3.txt\n");
	}
}

void PFSL3::updateCVCorners() {
	CvMat* t_point = cvCreateMat(1, 8, CV_64F);
	point_transform(rc_p, &mean_X_par[0], &p_transform[0], c_x, c_y, t_point);

	if(params.debug_mode){
		print_mat(rc_p, "rc_p", float, "%15.9f", debug_fid);
		print_mat(t_point, "t_point", float, "%15.9f", debug_fid);
		print_array(mean_X_par, 9, "mean_X_par", "%f", debug_fid);
		print_array(p_transform, 6, "p_transform", "%f", debug_fid);
	}
	cv_corners_mat.at<double>(0, 0) = (CV_MAT_ELEM(*t_point, double, 0, 1) + 0.5f) - 1;
	cv_corners_mat.at<double>(1, 0) = (CV_MAT_ELEM(*t_point, double, 0, 0) + 0.5f) - 1;
	cv_corners_mat.at<double>(0, 3) = (CV_MAT_ELEM(*t_point, double, 0, 3) + 0.5f) - 1;
	cv_corners_mat.at<double>(1, 3) = (CV_MAT_ELEM(*t_point, double, 0, 2) + 0.5f) + 1;
	cv_corners_mat.at<double>(0, 2) = (CV_MAT_ELEM(*t_point, double, 0, 5) + 0.5f) + 1;
	cv_corners_mat.at<double>(1, 2) = (CV_MAT_ELEM(*t_point, double, 0, 4) + 0.5f) + 1;
	cv_corners_mat.at<double>(0, 1) = (CV_MAT_ELEM(*t_point, double, 0, 7) + 0.5f) + 1;
	cv_corners_mat.at<double>(1, 1) = (CV_MAT_ELEM(*t_point, double, 0, 6) + 0.5f) - 1;
}

void PFSL3::setImage(const cv::Mat &_img){
	curr_img = _img;
	img = &curr_img;
}

void PFSL3::initialize(const cv::Mat&_cv_corners) {
	frame_id = 0;
	c_y = (_cv_corners.at<double>(0, 0) + _cv_corners.at<double>(0, 1) +
		_cv_corners.at<double>(0, 2) + _cv_corners.at<double>(0, 3)) / 4.0;
	c_x = (_cv_corners.at<double>(1, 0) + _cv_corners.at<double>(1, 1) +
		_cv_corners.at<double>(1, 2) + _cv_corners.at<double>(1, 3)) / 4.0;
	l_y = (abs(_cv_corners.at<double>(0, 0) - c_y) + abs(_cv_corners.at<double>(0, 1) - c_y)
		+ abs(_cv_corners.at<double>(0, 2) - c_y) + abs(_cv_corners.at<double>(0, 3) - c_y)) / 2.0;
	l_x = (abs(_cv_corners.at<double>(1, 0) - c_x) + abs(_cv_corners.at<double>(1, 1) - c_x)
		+ abs(_cv_corners.at<double>(1, 2) - c_x) + abs(_cv_corners.at<double>(1, 3) - c_x)) / 2.0;

	frame_size = cvGetSize(img);

	p_x = params.p_x;
	p_y = params.p_y;

	//Corner point coordinates centered on the rectangle
	c_p = cvCreateMat(3, 4, CV_64F);
	CV_MAT_ELEM(*c_p, double, 0, 0) = 1 - cvRound(((double)l_x) / 2);
	CV_MAT_ELEM(*c_p, double, 1, 0) = 1 - cvRound(((double)l_y) / 2);
	CV_MAT_ELEM(*c_p, double, 2, 0) = 1;
	CV_MAT_ELEM(*c_p, double, 0, 1) = l_x - cvRound(((double)l_x) / 2);
	CV_MAT_ELEM(*c_p, double, 1, 1) = 1 - cvRound(((double)l_y) / 2);
	CV_MAT_ELEM(*c_p, double, 2, 1) = 1;
	CV_MAT_ELEM(*c_p, double, 0, 2) = l_x - cvRound(((double)l_x) / 2);
	CV_MAT_ELEM(*c_p, double, 1, 2) = l_y - cvRound(((double)l_y) / 2);
	CV_MAT_ELEM(*c_p, double, 2, 2) = 1;
	CV_MAT_ELEM(*c_p, double, 0, 3) = 1 - cvRound(((double)l_x) / 2);
	CV_MAT_ELEM(*c_p, double, 1, 3) = l_y - cvRound(((double)l_y) / 2);
	CV_MAT_ELEM(*c_p, double, 2, 3) = 1;

	// Reduced corner point coordinates
	rc_p = cvCreateMat(3, 4, CV_64F);
	CV_MAT_ELEM(*rc_p, double, 0, 0) = 1 - cvRound(((double)p_x) / 2);
	CV_MAT_ELEM(*rc_p, double, 1, 0) = 1 - cvRound(((double)p_y) / 2);
	CV_MAT_ELEM(*rc_p, double, 2, 0) = 1;
	CV_MAT_ELEM(*rc_p, double, 0, 1) = p_x - cvRound(((double)p_x) / 2);
	CV_MAT_ELEM(*rc_p, double, 1, 1) = 1 - cvRound(((double)p_y) / 2);
	CV_MAT_ELEM(*rc_p, double, 2, 1) = 1;
	CV_MAT_ELEM(*rc_p, double, 0, 2) = p_x - cvRound(((double)p_x) / 2);
	CV_MAT_ELEM(*rc_p, double, 1, 2) = p_y - cvRound(((double)p_y) / 2);
	CV_MAT_ELEM(*rc_p, double, 2, 2) = 1;
	CV_MAT_ELEM(*rc_p, double, 0, 3) = 1 - cvRound(((double)p_x) / 2);
	CV_MAT_ELEM(*rc_p, double, 1, 3) = p_y - cvRound(((double)p_y) / 2);
	CV_MAT_ELEM(*rc_p, double, 2, 3) = 1;

	// Calculate the coordinate transformation matrix
	temp_cp = cvCreateMat(3, 3, CV_64F);
	cvGetSubRect(c_p, temp_cp, cvRect(0, 0, 3, 3));
	temp_rcp = cvCreateMat(3, 3, CV_64F);
	cvGetSubRect(rc_p, temp_rcp, cvRect(0, 0, 3, 3));
	inv_temp_rcp = cvCreateMat(3, 3, CV_64F);
	cvInvert(temp_rcp, inv_temp_rcp, CV_LU);
	transform = cvCreateMat(3, 3, CV_64F);
	cvMatMul(temp_cp, inv_temp_rcp, transform);
	memcpy(&p_transform[0], (double*)(transform->data.ptr), 6 * 8);

	if(params.debug_mode){
		fprintf(debug_fid, "l_x: %f l_y: %f\n", l_x, l_y);
		fprintf(debug_fid, "c_x: %f c_y: %f\n", c_x, c_y);
		print_mat(c_p, "c_p", double, "%15.9f", debug_fid);
		print_mat(temp_cp, "temp_cp", double, "%15.9f", debug_fid);
		print_mat(rc_p, "rc_p", double, "%15.9f", debug_fid);
		print_mat(temp_rcp, "temp_rcp", double, "%15.9f", debug_fid);
		print_mat(inv_temp_rcp, "inv_temp_rcp", double, "%15.9f", debug_fid);
		print_mat(transform, "transform", double, "%15.9f", debug_fid);
	}

	// Generating point matrix
	p_matrix = new double[2 * p_x * p_y];
	for(int i = 0; i < p_x; i++) {
		for(int j = 0; j < p_y; j++) {
			p_matrix[0 + (i * p_y + j) * 2] = i + 1 - cvRound(((double)p_x) / 2);
			p_matrix[1 + (i * p_y + j) * 2] = j + 1 - cvRound(((double)p_y) / 2);
		}
	}

	// Calculating the initial homography (rotation matrix)
	init_h[0] = cos(params.rot / 180 * CV_PI);
	init_h[1] = sin(params.rot / 180 * CV_PI);
	init_h[2] = 0;
	init_h[3] = -sin(params.rot / 180 * CV_PI);
	init_h[4] = cos(params.rot / 180 * CV_PI);
	init_h[5] = 0;
	init_h[6] = 0;
	init_h[7] = 0;
	init_h[8] = 1;

	// Extracting object template (p_x by p_y size)
	obj_template = cvCreateMat(1, p_x * p_y, CV_32F);
	imwarping_BL(img, &init_h[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, obj_template);
	warped_img_2D = cvReshape(obj_template, &warped_img_2D_header, 0, p_x);
	//cvNamedWindow("Initial Templates");
	//cvShowImage("Initial Templates", warped_img_2D);

	tracked_img = new float[p_x * p_y * params.len];
	memcpy(&tracked_img[0], (float*)(obj_template->data.ptr), p_x * p_y * 4);
	num_tracked_img = 1;

	// Precompute the requisite for Jacobian
	dX_du = cvCreateMat(9, 8, CV_32F);
	cvSetZero(dX_du);
	CV_MAT_ELEM(*dX_du, float, 0, 4) = -1;
	CV_MAT_ELEM(*dX_du, float, 1, 2) = -1;
	CV_MAT_ELEM(*dX_du, float, 1, 3) = -1;
	CV_MAT_ELEM(*dX_du, float, 2, 6) = -1;
	CV_MAT_ELEM(*dX_du, float, 3, 2) = 1;
	CV_MAT_ELEM(*dX_du, float, 3, 3) = -1;
	CV_MAT_ELEM(*dX_du, float, 4, 4) = 1;
	CV_MAT_ELEM(*dX_du, float, 4, 5) = 1;
	CV_MAT_ELEM(*dX_du, float, 5, 7) = -1;
	CV_MAT_ELEM(*dX_du, float, 6, 0) = -1;
	CV_MAT_ELEM(*dX_du, float, 7, 1) = -1;
	CV_MAT_ELEM(*dX_du, float, 8, 5) = -1;

	// Calculate the gradients of template
	grad_x = cvCreateMat(p_x, p_y, CV_32F);
	grad_y = cvCreateMat(p_x, p_y, CV_32F);
	image_gradient(warped_img_2D, p_x, p_y, grad_x, grad_y);

	// Jacobian calculation
	jacobian = cvCreateMat(p_x * p_y, 8, CV_32F);
	jacobian_calculation(p_matrix, grad_x, grad_y, dX_du, p_x, p_y, jacobian);

	// Mean-adjusted template
	adj_template = cvCreateMat(1, p_x * p_y, CV_32F);
	mean_v = cvAvg(obj_template);
	cvSubS(obj_template, mean_v, adj_template);

	// Initialize PF variables
	if(params.sampling == 0) {
		params.N = params.N * params.N_c;
		params.N_c = 1;
	}
	X_par = new double[params.N * 9];
	X_pred = new double[params.N * params.N_c * 9];
	AR_velocity = new double[params.N * 9];
	AR_pred = new double[params.N * params.N_c * 9];
	mean_X_par = new double[9];
	prob = new double[params.N * params.N_c];
	w = new double[params.N * params.N_c];

	// Initialize state particles at t = 0;
	for(int i = 0; i < params.N; i++) {
		memcpy(&X_par[i * 9], &init_h[0], 9 * 8);
		AR_velocity[0 + i * 9] = 1;
		AR_velocity[1 + i * 9] = 0;
		AR_velocity[2 + i * 9] = 0;
		AR_velocity[3 + i * 9] = 0;
		AR_velocity[4 + i * 9] = 1;
		AR_velocity[5 + i * 9] = 0;
		AR_velocity[6 + i * 9] = 0;
		AR_velocity[7 + i * 9] = 0;
		AR_velocity[8 + i * 9] = 1;
	}
	memcpy(&mean_X_par[0], &init_h[0], 9 * 8);

	// Setup for incremental PCA
	basis = cvCreateMat(params.max_num_basis, p_x * p_y, CV_32F);
	projected = cvCreateMat(1, params.max_num_used_basis, CV_32F);
	data_proj = cvCreateMat(params.update_period + 1, params.max_num_basis, CV_32F);
	diff_img = cvCreateMat(1, p_x * p_y, CV_32F);
	temp_basis = cvCreateMat(1, p_x * p_y, CV_32F);

	mean_img = new float[p_x * p_y];
	mean_update_img = new float[p_x * p_y];
	cv_mean_img = cvCreateMat(1, p_x * p_y, CV_32F);
	mean_adj_img = new float[p_x * p_y * (MMAX(params.init_size, params.update_period))];
	update_img = new float[p_x * p_y * params.update_period];
	new_data = new float[p_x * p_y * (params.update_period + 1)];
	data_res = cvCreateMat(params.update_period + 1, p_x * p_y, CV_32F);
	num_update_img = 0;
	PCA_flag = 0;

	divide = cvCreateMat(p_x, p_y, CV_32F);
	if(params.show_templates){
		// Template images
		templ = cvCreateMat(p_x, p_y * (3 + params.max_num_used_basis), CV_32F);
		cvSetZero(templ);		
		for(int i = 0; i < p_x; i++) {
			for(int j = 0; j < p_y; j++) {
				CV_MAT_ELEM(*templ, float, i, j) = CV_MAT_ELEM(*warped_img_2D, float, i, j);
				CV_MAT_ELEM(*templ, float, i, j + p_y) = CV_MAT_ELEM(*warped_img_2D, float, i, j);
				CV_MAT_ELEM(*templ, float, i, j + p_y + p_y) = CV_MAT_ELEM(*warped_img_2D, float, i, j);
			}
		}
		cvNamedWindow("Templates");
		cvShowImage("Templates", templ);
	}

	// Execute tracking
	rng_state = cvRNG(GetRDTSC());
	random = cvCreateMat(1, 8, CV_32F);
	previous_X3x3 = cvCreateMat(3, 3, CV_64F);
	warped_img = cvCreateMat(1, p_x * p_y, CV_32F);
	adj_warped_img = cvCreateMat(1, p_x * p_y, CV_32F);
	divide_prob = cvCreateMat(1, params.N, CV_64F);


	outindex = new int[params.N];
	temp_MAT = cvCreateMat(1, 9, CV_64F);
	temp_w = cvCreateMat(1, params.N * params.N_c, CV_64F);
	for(int i = 0; i < params.N; i++) {
		outindex[i] = 0;
	}

	Mean = new double[9 * (params.N_iter + 1)];
	Cov = new double[64 * (params.N_iter + 1)];
	for(int i = 0; i < 64; i++) {
		state_cov[i] = 0;
	}
	state_cov[0] = params.state_std[0] * params.state_std[0];
	state_cov[9] = params.state_std[1] * params.state_std[1];
	state_cov[18] = params.state_std[2] * params.state_std[2];
	state_cov[27] = params.state_std[3] * params.state_std[3];
	state_cov[36] = params.state_std[4] * params.state_std[4];
	state_cov[45] = params.state_std[5] * params.state_std[5];
	state_cov[54] = params.state_std[6] * params.state_std[6];
	state_cov[63] = params.state_std[7] * params.state_std[7];
	Init_Cov = cvCreateMat(8, 8, CV_32F);
	cvSetZero(Init_Cov);
	CV_MAT_ELEM(*Init_Cov, float, 0, 0) = state_cov[0];
	CV_MAT_ELEM(*Init_Cov, float, 1, 1) = state_cov[9];
	CV_MAT_ELEM(*Init_Cov, float, 2, 2) = state_cov[18];
	CV_MAT_ELEM(*Init_Cov, float, 3, 3) = state_cov[27];
	CV_MAT_ELEM(*Init_Cov, float, 4, 4) = state_cov[36];
	CV_MAT_ELEM(*Init_Cov, float, 5, 5) = state_cov[45];
	CV_MAT_ELEM(*Init_Cov, float, 6, 6) = state_cov[54];
	CV_MAT_ELEM(*Init_Cov, float, 7, 7) = state_cov[63];

	NCC_real = 1;
	PCA_real = 0;

	pred_measure = new double[params.N_iter + 1];
	NCC_jacobian = cvCreateMat(1, 8, CV_32F);
	NCC_J_temp1 = cvCreateMat(1, 8, CV_32F);
	NCC_J_temp2 = cvCreateMat(1, 8, CV_32F);
	sigma_11 = cvCreateMat(8, 8, CV_32F);
	sigma_12 = cvCreateMat(8, 1, CV_32F);
	sigma_22 = cvCreateMat(1, 1, CV_32F);
	temp_like_NCC = cvCreateMat(1, 1, CV_32F);
	CV_MAT_ELEM(*temp_like_NCC, float, 0, 0) = params.NCC_std * params.NCC_std;

	inv_sigma_22 = cvCreateMat(1, 1, CV_32F);
	temp_Cov1 = cvCreateMat(8, 1, CV_32F);
	temp_Cov2 = cvCreateMat(8, 8, CV_32F);
	temp_increment = cvCreateMat(8, 1, CV_32F);
	update_NCC_thr = 0.3;
	temp_pred_measure = cvCreateMat(1, params.N_iter + 1, CV_64F);

	UPLO = 'L';

	CH = cvCreateMat(8, 8, CV_32F);
	cvSetZero(CH);
	colored_random = cvCreateMat(1, 8, CV_32F);
	dist3_temp1 = cvCreateMat(1, 8, CV_32F);
	dist3_temp2 = cvCreateMat(1, 1, CV_32F);
	COV3 = cvCreateMat(8, 8, CV_32F);
	inv_COV3 = cvCreateMat(8, 8, CV_32F);

	dist2_temp1 = cvCreateMat(1, 8, CV_32F);
	dist2_temp2 = cvCreateMat(1, 1, CV_32F);
	COV2 = cvCreateMat(8, 8, CV_32F);
	inv_COV2 = cvCreateMat(8, 8, CV_32F);

	X_diff_cv = cvCreateMat(1, 8, CV_32F);
	PCA_recon = cvCreateMat(1, p_x * p_y, CV_32F);
	cvSetZero(PCA_recon);
	PCA_recon_weight = cvCreateMat(1, 1, CV_32F);
	PCA_recon_temp = cvCreateMat(1, p_x * p_y, CV_32F);
	mean_jacobian = cvCreateMat(p_x * p_y, 8, CV_32F);
	d_PCA_measure = cvCreateMat(1, p_x * p_y, CV_32F);

	PCA_jacobian = cvCreateMat(1, 8, CV_32F);
	total_jacobian = cvCreateMat(2, 8, CV_32F);
	t_sigma_12 = cvCreateMat(8, 2, CV_32F);
	t_sigma_22 = cvCreateMat(2, 2, CV_32F);
	total_like = cvCreateMat(2, 2, CV_32F);

	cvSetZero(total_like);
	CV_MAT_ELEM(*total_like, float, 0, 0) = params.NCC_std * params.NCC_std;
	CV_MAT_ELEM(*total_like, float, 1, 1) = params.PCA_std * params.PCA_std;
	inv_t_sigma_22 = cvCreateMat(2, 2, CV_32F);
	t_temp_Cov1 = cvCreateMat(8, 2, CV_32F);

	update_PCA_thr = -20;
	num_est = 0;
	if(params.show_weights){
		weight_plot = cvCreateMat(50, params.N * params.N_c, CV_32F);
		cvSetZero(weight_plot);
		cvNamedWindow("Particle Weights");
	}
	outlier_thr = 0.25;

	recon_error = cvCreateMat(1, p_x * p_y, CV_32F);

	QR_Q = cvCreateMat(params.update_period + 1, p_x * p_y, CV_32F);
	Q_proj = cvCreateMat(params.update_period + 1, params.update_period + 1, CV_32F);
	W = cvCreateMat(params.init_size, params.init_size, CV_32F);
	U = cvCreateMat(params.init_size, params.init_size, CV_32F);
	V = cvCreateMat(params.init_size, p_x * p_y, CV_32F);
	A = cvCreateMat(params.init_size, p_x * p_y, CV_32F);
	Sing_val = cvCreateMat(1, params.max_num_basis, CV_32F);

	temp_img = new float[p_x * p_y];
	num_outlier = 0;
	update_thr = 0.05;

	bestindex_plot = cvCreateMat((params.N_iter + 1) * 5, params.N * 5, CV_32F);
	cvSetZero(bestindex_plot);

	outindex2 = new int[params.N * params.N_c];
	X_par_mean = new double[9 * params.N * params.N_c];
	AR_velocity_mean = new double[9 * params.N * params.N_c];
	new_NCC_sig = new double[params.N_iter + 1];
	_cv_corners.copyTo(cv_corners_mat);
}

void PFSL3::update() {
	//printf("starting update\n");
	integer order;
	order = 8;
	integer info;
	doublereal Covariance[8][8];
	doublereal* temp_res;
	doublereal* tau;
	doublereal* WORK;
	integer qr_M, qr_N, qr_LDA, qr_LWORK, qr_INFO;
	//printf("starting temp_res with p_x: %d p_y: %d params.update_period: %d", p_x, p_y, params.update_period);
	temp_res = new doublereal[(p_x * p_y) * (params.update_period + 1)];
	//printf("done temp_res");
	tau = new doublereal[params.update_period + 1];
	//printf("done tau");
	qr_M = p_x * p_y;
	//printf("done qr_M");
	qr_N = params.update_period + 1;
	//printf("done qr_N");
	qr_LDA = p_x * p_y;
	//printf("done qr_LDA");
	qr_LWORK = params.update_period + 1;
	//printf("done qr_LWORK");
	WORK = new doublereal[params.update_period + 1];
	//printf("done WORK");

	++frame_id;
	if(frame_id >= params.len){
		frame_id = 0;
	}
	int previous_index = -1;
	for(int par = 0; par < params.N; par++) {
		//printf("Processing particle %d\n", par);
		if(params.sampling == 0) {
			if(PCA_flag == 0) {
				memcpy(&previous_X, &X_par[par * 9], 9 * 8);
				memcpy(&AR, &AR_velocity[par * 9], 9 * 8);

				mat_log(&AR[0], &log_AR[0]);
				mat_scalar(&log_AR[0], params.AR_p, &a_log_AR[0]);

				for(int row = 0; row < 8; row++) {
					for(int col = 0; col < 8; col++) {
						Covariance[row][col] = state_cov[row * 8 + col];
					}
				}
				dpotrf_(&UPLO, &order, &Covariance[0][0], &order, &info);
				for(int row = 0; row < 8; row++) {
					for(int col = row; col < 8; col++) {
						CV_MAT_ELEM(*CH, float, row, col) = (float)Covariance[row][col];
					}
				}
				cvRandArr(&rng_state, random, CV_RAND_NORMAL, cvScalar(0), cvScalar(1));
				cvGEMM(random, CH, 1, NULL, 0, colored_random);
				for(int num = 0; num < 8; num++) {
					scaled_rand[num] = (double)CV_MAT_ELEM(*colored_random, float, 0, num);
				}
				sl3(&scaled_rand[0], &sl3_rand[0]);
				mat_add(&a_log_AR[0], &sl3_rand[0], &_update[0]);
				mat_exp(&_update[0], &update_exp[0]);
				mat_mul_3x3(&previous_X[0], &update_exp[0], &pred_X[0]);
				memcpy(&X_pred[par * 9], &pred_X[0], 9 * 8);
				mat_inv_3x3(&previous_X[0], &inv_X[0]);
				mat_mul_3x3(&inv_X[0], &pred_X[0], &pred_AR[0]);
				memcpy(&AR_pred[par * 9], &pred_AR[0], 9 * 8);

				if(params.debug_mode){
					print_mat(CH, "CH", float, "%15.9f", debug_fid);
					print_mat(random, "random", float, "%15.9f", debug_fid);
					print_array(scaled_rand, 8, "scaled_rand", "%f", debug_fid);
					print_array(sl3_rand, 9, "sl3_rand", "%f", debug_fid);
					print_array(a_log_AR, 9, "a_log_AR", "%f", debug_fid);
					print_array(_update, 9, "_update", "%f", debug_fid);
					print_array(update_exp, 9, "update_exp", "%f", debug_fid);
					print_array(pred_X, 9, "pred_X", "%f", debug_fid);
					print_array(previous_X, 9, "previous_X", "%f", debug_fid);
					print_array(inv_X, 9, "inv_X", "%f", debug_fid);
					print_array(pred_AR, 9, "pred_AR", "%f", debug_fid);
				}

				imwarping_NN(img, &pred_X[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
				mean_v = cvAvg(warped_img);
				cvSubS(warped_img, mean_v, adj_warped_img);
				u = cvDotProduct(adj_warped_img, adj_template);
				v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
				NCC = u / v;

				dist = MMAX(-0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std, -250);
				prob[par] = exp(dist);
			} else {
				memcpy(&previous_X, &X_par[par * 9], 9 * 8);
				memcpy(&AR, &AR_velocity[par * 9], 9 * 8);

				mat_log(&AR[0], &log_AR[0]);
				mat_scalar(&log_AR[0], params.AR_p, &a_log_AR[0]);

				mat_log(&AR[0], &log_AR[0]);
				mat_scalar(&log_AR[0], params.AR_p, &a_log_AR[0]);

				for(int row = 0; row < 8; row++) {
					for(int col = 0; col < 8; col++) {
						Covariance[row][col] = state_cov[row * 8 + col];
					}
				}
				dpotrf_(&UPLO, &order, &Covariance[0][0], &order, &info);
				for(int row = 0; row < 8; row++) {
					for(int col = row; col < 8; col++) {
						CV_MAT_ELEM(*CH, float, row, col) = (float)Covariance[row][col];
					}
				}
				cvRandArr(&rng_state, random, CV_RAND_NORMAL, cvScalar(0), cvScalar(1));
				cvGEMM(random, CH, 1, NULL, 0, colored_random);
				for(int num = 0; num < 8; num++) {
					scaled_rand[num] = (double)CV_MAT_ELEM(*colored_random, float, 0, num);
				}
				sl3(&scaled_rand[0], &sl3_rand[0]);

				mat_add(&a_log_AR[0], &sl3_rand[0], &_update[0]);
				mat_exp(&_update[0], &update_exp[0]);
				mat_mul_3x3(&previous_X[0], &update_exp[0], &pred_X[0]);
				memcpy(&X_pred[par * 9], &pred_X[0], 9 * 8);
				mat_inv_3x3(&previous_X[0], &inv_X[0]);
				mat_mul_3x3(&inv_X[0], &pred_X[0], &pred_AR[0]);
				memcpy(&AR_pred[par * 9], &pred_AR[0], 9 * 8);

				imwarping_NN(img, &pred_X[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
				mean_v = cvAvg(warped_img);
				cvSubS(warped_img, mean_v, adj_warped_img);

				if(params.outlier_flag == 0) {
					u = cvDotProduct(adj_warped_img, adj_template);
					v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
					NCC = u / v;

					cvSub(cv_mean_img, warped_img, diff_img);
					proj_sum = 0;
					for(int i = 0; i < num_used_basis; i++) {
						memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
						CV_MAT_ELEM(*projected, float, 0, i) = cvDotProduct(temp_basis, diff_img);
						proj_sum = proj_sum + CV_MAT_ELEM(*projected, float, 0, i) * CV_MAT_ELEM(*projected, float, 0, i);
					}
					PCA = cvDotProduct(diff_img, diff_img) - proj_sum;
				} else {
					cvSub(cv_mean_img, warped_img, diff_img);
					proj_sum = 0;
					cvSetZero(PCA_recon);
					for(int i = 0; i < num_used_basis; i++) {
						memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
						CV_MAT_ELEM(*projected, float, 0, i) = cvDotProduct(temp_basis, diff_img);
						proj_sum = proj_sum + CV_MAT_ELEM(*projected, float, 0, i) * CV_MAT_ELEM(*projected, float, 0, i);

						CV_MAT_ELEM(*PCA_recon_weight, float, 0, 0) = CV_MAT_ELEM(*projected, float, 0, i);
						cvGEMM(PCA_recon_weight, temp_basis, 1, NULL, 0, PCA_recon_temp);
						cvAdd(PCA_recon_temp, PCA_recon, PCA_recon);
					}
					PCA = cvDotProduct(diff_img, diff_img) - proj_sum;

					cvSub(diff_img, PCA_recon, recon_error);
					for(int num = 0; num < p_x * p_y; num++) {
						if(abs(CV_MAT_ELEM(*recon_error, float, 0, num)) > outlier_thr) {
							CV_MAT_ELEM(*adj_warped_img, float, 0, num) = 0;
						}
					}
					u = cvDotProduct(adj_warped_img, adj_template);
					v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
					NCC = u / v;
				}

				dist = MMAX(-0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std - 0.5 * (PCA_real - PCA) * (PCA_real - PCA) / params.PCA_std / params.PCA_std, -250);
				prob[par] = exp(dist);
			}
		} else {
			if(PCA_flag == 0) {
				if(previous_index != outindex[par]) {
					num_est++;
					memcpy(&previous_X, &X_par[par * 9], 9 * 8);
					memcpy(&AR, &AR_velocity[par * 9], 9 * 8);

					mat_log(&AR[0], &log_AR[0]);
					mat_scalar(&log_AR[0], params.AR_p, &a_log_AR[0]);

					mat_exp(&a_log_AR[0], &update_exp[0]);
					mat_mul_3x3(&previous_X[0], &update_exp[0], &X_par_temp[0]);

					imwarping_NN(img, &X_par_temp[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
					mean_v = cvAvg(warped_img);
					cvSubS(warped_img, mean_v, adj_warped_img);
					u = cvDotProduct(adj_warped_img, adj_template);
					v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
					NCC = u / v;

					memcpy(&Mean[0], &X_par_temp[0], 9 * 8);
					memcpy(&Cov[0], &state_cov[0], 64 * 8);

					diff1 = -0.5 * (NCC_real - NCC) * (NCC_real - NCC) / CV_MAT_ELEM(*temp_like_NCC, float, 0, 0);
					pred_measure[0] = exp(diff1);
					for(int iter = 1; iter < params.N_iter + 1; iter++) {
						cvGEMM(adj_warped_img, jacobian, 1 / v, NULL, 0, NCC_J_temp1);
						cvGEMM(adj_template, jacobian, (-u) / (v * v * v)*cvDotProduct(adj_warped_img, adj_warped_img), NULL, 0, NCC_J_temp2);
						cvAdd(NCC_J_temp1, NCC_J_temp2, NCC_jacobian);

						mu_2 = NCC;
						for(int row = 0; row < 8; row++) {
							for(int col = 0; col < 8; col++) {
								CV_MAT_ELEM(*sigma_11, float, row, col) = Cov[64 * (iter - 1) + row * 8 + col];
							}
						}

						cvGEMM(sigma_11, NCC_jacobian, 1, NULL, 0, sigma_12, CV_GEMM_B_T);
						cvGEMM(NCC_jacobian, sigma_12, 1, temp_like_NCC, 1, sigma_22);
						cvInvert(sigma_22, inv_sigma_22);
						cvMatMul(sigma_12, inv_sigma_22, temp_Cov1);
						cvGEMM(temp_Cov1, sigma_12, -1, sigma_11, 1, temp_Cov2, CV_GEMM_B_T);
						for(int row = 0; row < 8; row++) {
							for(int col = 0; col < 8; col++) {
								Cov[64 * iter + row * 8 + col] = CV_MAT_ELEM(*temp_Cov2, float, row, col);
							}
						}

						cvGEMM(sigma_12, inv_sigma_22, MMIN(NCC_real - mu_2, update_NCC_thr), NULL, 0, temp_increment);
						for(int num = 0; num < 8; num++) {
							increment[num] = CV_MAT_ELEM(*temp_increment, float, num, 0);
						}
						sl3(&increment[0], &log_AR[0]);
						mat_exp(&log_AR[0], &update_exp[0]);
						mat_mul_3x3(&Mean[9 * (iter - 1)], &update_exp[0], &Mean[9 * iter]);

						imwarping_NN(img, &Mean[9 * iter], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
						mean_v = cvAvg(warped_img);
						cvSubS(warped_img, mean_v, adj_warped_img);
						u = cvDotProduct(adj_warped_img, adj_template);
						v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
						NCC = u / v;

						diff1 = -0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std;
						mat_inv_3x3(&X_par_temp[0], &inv_X[0]);
						mat_mul_3x3(&inv_X[0], &Mean[9 * iter], &pred_AR[0]);
						mat_log(&pred_AR[0], &log_AR[0]);
						sl3_to_vec(&log_AR[0], &X_diff[0]);
						diff2 = -0.5 * (X_diff[0] * X_diff[0] / state_cov[0] + X_diff[1] * X_diff[1] / state_cov[9] + X_diff[2] * X_diff[2] / state_cov[18] + X_diff[3] * X_diff[3] / state_cov[27] + X_diff[4] * X_diff[4] / state_cov[36] + X_diff[5] * X_diff[5] / state_cov[45] + X_diff[6] * X_diff[6] / state_cov[54] + X_diff[7] * X_diff[7] / state_cov[63]);
						pred_measure[iter] = exp(diff1 + diff2);
					}
					memcpy((double*)(temp_pred_measure->data.ptr), &pred_measure[0], (params.N_iter + 1) * 8);
					cvMinMaxLoc(temp_pred_measure, &min_v, &max_v, &min_ind, &max_ind);
					best_index = max_ind.x;
					//best_index = 1;
				} // if(previous_index != outindex[par])
				memcpy(&New_Cov[0], &Cov[64 * best_index], 64 * 8);
				memcpy(&New_Mean[0], &Mean[9 * best_index], 9 * 8);
				if(best_index == 0) {
					memcpy(&Prev_Cov[0], &Cov[64 * best_index], 64 * 8);
					memcpy(&Prev_Mean[0], &Mean[9 * best_index], 9 * 8);
				} else {
					memcpy(&Prev_Cov[0], &Cov[64 * (best_index - 1)], 64 * 8);
					memcpy(&Prev_Mean[0], &Mean[9 * (best_index - 1)], 9 * 8);
				}
				CV_MAT_ELEM(*sigma_22, float, 0, 0) = (float)new_NCC_sig[best_index];

				for(int ppp = 0; ppp < best_index + 1; ppp++) {
					for(int pp = 0; pp < 5; pp++) {
						for(int pppp = 0; pppp < 5; pppp++) {
							CV_MAT_ELEM(*bestindex_plot, float, 5 * (params.N_iter + 1) - 1 - (ppp * 5 + pppp), par * 5 + pp) = 1;
						}
					}
				}

				for(int n_inner = params.N_c * par; n_inner < params.N_c * (par + 1); n_inner++) {
					for(int row = 0; row < 8; row++) {
						for(int col = 0; col < 8; col++) {
							Covariance[row][col] = New_Cov[row * 8 + col];
						}
					}
					dpotrf_(&UPLO, &order, &Covariance[0][0], &order, &info);
					for(int row = 0; row < 8; row++) {
						for(int col = row; col < 8; col++) {
							CV_MAT_ELEM(*CH, float, row, col) = (float)Covariance[row][col];
						}
					}

					cvRandArr(&rng_state, random, CV_RAND_NORMAL, cvScalar(0), cvScalar(1));
					cvGEMM(random, CH, 1, NULL, 0, colored_random);
					for(int num = 0; num < 8; num++) {
						scaled_rand[num] = (double)CV_MAT_ELEM(*colored_random, float, 0, num);
					}
					sl3(&scaled_rand[0], &sl3_rand[0]);
					mat_exp(&sl3_rand[0], &update_exp[0]);

					memcpy(&previous_X, &New_Mean[0], 9 * 8);
					mat_mul_3x3(&previous_X[0], &update_exp[0], &pred_X[0]);
					memcpy(&X_pred[n_inner * 9], &pred_X[0], 9 * 8);
					mat_inv_3x3(&X_par[par * 9], &inv_X[0]);
					mat_mul_3x3(&inv_X[0], &pred_X[0], &pred_AR[0]);
					memcpy(&AR_pred[n_inner * 9], &pred_AR[0], 9 * 8);

					imwarping_NN(img, &pred_X[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
					mean_v = cvAvg(warped_img);
					cvSubS(warped_img, mean_v, adj_warped_img);
					u = cvDotProduct(adj_warped_img, adj_template);
					v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
					NCC = u / v;

					dist1 = -0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std;
					mat_inv_3x3(&Prev_Mean[0], &inv_X[0]);
					mat_mul_3x3(&inv_X[0], &pred_X[0], &pred_AR[0]);
					mat_log(&pred_AR[0], &log_AR[0]);
					sl3_to_vec(&log_AR[0], &X_diff[0]);
					for(int row = 0; row < 8; row++) {
						for(int col = 0; col < 8; col++) {
							CV_MAT_ELEM(*COV2, float, row, col) = (float)Prev_Cov[row * 8 + col];
						}
					}
					cvInvert(COV2, inv_COV2);
					for(int num = 0; num < 8; num++) {
						CV_MAT_ELEM(*X_diff_cv, float, 0, num) = (float)X_diff[num];
					}
					cvGEMM(X_diff_cv, inv_COV2, -0.5, NULL, 0, dist2_temp1);
					cvGEMM(dist2_temp1, X_diff_cv, 1, NULL, 0, dist2_temp2, CV_GEMM_B_T);
					dist2 = (double)CV_MAT_ELEM(*dist2_temp2, float, 0, 0);

					for(int row = 0; row < 8; row++) {
						for(int col = 0; col < 8; col++) {
							CV_MAT_ELEM(*COV3, float, row, col) = (float)New_Cov[row * 8 + col];
						}
					}
					cvInvert(COV3, inv_COV3);
					cvGEMM(colored_random, inv_COV3, -0.5, NULL, 0, dist3_temp1);
					cvGEMM(dist3_temp1, colored_random, 1, NULL, 0, dist3_temp2, CV_GEMM_B_T);
					dist3 = (double)CV_MAT_ELEM(*dist3_temp2, float, 0, 0);
					dist = MMAX(dist1 + dist2 - dist3, -250);
					prob[n_inner] = exp(dist) * (1 / ((float)cvSqrt((double)cvDet(COV2)))) / (1 / ((float)cvSqrt((double)cvDet(COV3))));
				}
				previous_index = outindex[par];
			} else { // if(PCA_flag == 0)
				if(params.outlier_flag == 0) {
					if(previous_index != outindex[par]) {
						num_est++;
						memcpy(&previous_X, &X_par[par * 9], 9 * 8);
						memcpy(&AR, &AR_velocity[par * 9], 9 * 8);

						mat_log(&AR[0], &log_AR[0]);
						mat_scalar(&log_AR[0], params.AR_p, &a_log_AR[0]);

						mat_exp(&a_log_AR[0], &update_exp[0]);
						mat_mul_3x3(&previous_X[0], &update_exp[0], &X_par_temp[0]);

						imwarping_NN(img, &X_par_temp[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
						mean_v = cvAvg(warped_img);
						cvSubS(warped_img, mean_v, adj_warped_img);
						u = cvDotProduct(adj_warped_img, adj_template);
						v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
						NCC = u / v;

						cvSub(cv_mean_img, warped_img, diff_img);
						proj_sum = 0;
						cvSetZero(PCA_recon);
						for(int i = 0; i < num_used_basis; i++) {
							memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
							CV_MAT_ELEM(*projected, float, 0, i) = cvDotProduct(temp_basis, diff_img);
							proj_sum = proj_sum + CV_MAT_ELEM(*projected, float, 0, i) * CV_MAT_ELEM(*projected, float, 0, i);

							CV_MAT_ELEM(*PCA_recon_weight, float, 0, 0) = CV_MAT_ELEM(*projected, float, 0, i);
							cvGEMM(PCA_recon_weight, temp_basis, 1, NULL, 0, PCA_recon_temp);
							cvAdd(PCA_recon_temp, PCA_recon, PCA_recon);
						}
						PCA = cvDotProduct(diff_img, diff_img) - proj_sum;

						memcpy(&Mean[0], &X_par_temp[0], 9 * 8);
						memcpy(&Cov[0], &state_cov[0], 64 * 8);

						diff1 = -0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std - 0.5 * (PCA_real - PCA) * (PCA_real - PCA) / params.PCA_std / params.PCA_std;
						pred_measure[0] = exp(diff1);

						for(int iter = 1; iter < params.N_iter + 1; iter++) {
							cvGEMM(adj_warped_img, jacobian, 1 / v, NULL, 0, NCC_J_temp1);
							cvGEMM(adj_template, jacobian, (-u) / (v * v * v)*cvDotProduct(adj_warped_img, adj_warped_img), NULL, 0, NCC_J_temp2);
							cvAdd(NCC_J_temp1, NCC_J_temp2, NCC_jacobian);

							cvSub(diff_img, PCA_recon, d_PCA_measure);
							CV_MAT_ELEM(*PCA_recon_weight, float, 0, 0) = 1;
							cvGEMM(PCA_recon_weight, d_PCA_measure, 2, NULL, 0, d_PCA_measure);
							cvGEMM(d_PCA_measure, mean_jacobian, 1, NULL, 0, PCA_jacobian);

							t_mu_2[0] = NCC;
							t_mu_2[1] = PCA;
							for(int row = 0; row < 8; row++) {
								for(int col = 0; col < 8; col++) {
									CV_MAT_ELEM(*sigma_11, float, row, col) = Cov[64 * (iter - 1) + row * 8 + col];
								}
							}

							for(int num = 0; num < 8; num++) {
								CV_MAT_ELEM(*total_jacobian, float, 0, num) = CV_MAT_ELEM(*NCC_jacobian, float, 0, num);
								CV_MAT_ELEM(*total_jacobian, float, 1, num) = CV_MAT_ELEM(*PCA_jacobian, float, 0, num);
							}

							cvGEMM(sigma_11, total_jacobian, 1, NULL, 0, t_sigma_12, CV_GEMM_B_T);
							cvMatMul(total_jacobian, t_sigma_12, t_sigma_22);
							cvAdd(t_sigma_22, total_like, t_sigma_22);

							cvInvert(t_sigma_22, inv_t_sigma_22);
							cvMatMul(t_sigma_12, inv_t_sigma_22, t_temp_Cov1);
							cvGEMM(t_temp_Cov1, t_sigma_12, -1, sigma_11, 1, temp_Cov2, CV_GEMM_B_T);
							for(int row = 0; row < 8; row++) {
								for(int col = 0; col < 8; col++) {
									Cov[64 * iter + row * 8 + col] = CV_MAT_ELEM(*temp_Cov2, float, row, col);
								}
							}

							CvMat* temp_diff = cvCreateMat(2, 1, CV_32F);
							CV_MAT_ELEM(*temp_diff, float, 0, 0) = MMIN(NCC_real - t_mu_2[0], update_NCC_thr);
							CV_MAT_ELEM(*temp_diff, float, 1, 0) = MMAX(PCA_real - t_mu_2[1], update_PCA_thr);
							cvMatMul(t_temp_Cov1, temp_diff, temp_increment);
							for(int num = 0; num < 8; num++) {
								increment[num] = CV_MAT_ELEM(*temp_increment, float, num, 0);
							}

							sl3(&increment[0], &log_AR[0]);
							mat_exp(&log_AR[0], &update_exp[0]);
							mat_mul_3x3(&Mean[9 * (iter - 1)], &update_exp[0], &Mean[9 * iter]);

							imwarping_NN(img, &Mean[9 * iter], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
							mean_v = cvAvg(warped_img);
							cvSubS(warped_img, mean_v, adj_warped_img);
							u = cvDotProduct(adj_warped_img, adj_template);
							v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
							NCC = u / v;

							cvSub(cv_mean_img, warped_img, diff_img);
							proj_sum = 0;
							cvSetZero(PCA_recon);
							for(int i = 0; i < num_used_basis; i++) {
								memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
								CV_MAT_ELEM(*projected, float, 0, i) = cvDotProduct(temp_basis, diff_img);
								proj_sum = proj_sum + CV_MAT_ELEM(*projected, float, 0, i) * CV_MAT_ELEM(*projected, float, 0, i);

								CV_MAT_ELEM(*PCA_recon_weight, float, 0, 0) = CV_MAT_ELEM(*projected, float, 0, i);
								cvGEMM(PCA_recon_weight, temp_basis, 1, NULL, 0, PCA_recon_temp);
								cvAdd(PCA_recon_temp, PCA_recon, PCA_recon);
							}
							PCA = cvDotProduct(diff_img, diff_img) - proj_sum;

							diff1 = -0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std - 0.5 * (PCA_real - PCA) * (PCA_real - PCA) / params.PCA_std / params.PCA_std;
							cvInvert(sigma_11, inv_COV2);
							cvGEMM(temp_increment, inv_COV2, -0.5, NULL, 0, dist2_temp1, CV_GEMM_A_T);
							cvGEMM(dist2_temp1, temp_increment, 1, NULL, 0, dist2_temp2);
							diff2 = (double)CV_MAT_ELEM(*dist2_temp2, float, 0, 0);
							pred_measure[iter] = exp(diff1 + diff2);
						}
						memcpy((double*)(temp_pred_measure->data.ptr), &pred_measure[0], (params.N_iter + 1) * 8);
						cvMinMaxLoc(temp_pred_measure, &min_v, &max_v, &min_ind, &max_ind);
						best_index = max_ind.x;
						//best_index = 1;
					}
					memcpy(&New_Cov[0], &Cov[64 * best_index], 64 * 8);
					memcpy(&New_Mean[0], &Mean[9 * best_index], 9 * 8);
					if(best_index == 0) {
						memcpy(&Prev_Cov[0], &Cov[64 * best_index], 64 * 8);
						memcpy(&Prev_Mean[0], &Mean[9 * best_index], 9 * 8);
					} else {
						memcpy(&Prev_Cov[0], &Cov[64 * (best_index - 1)], 64 * 8);
						memcpy(&Prev_Mean[0], &Mean[9 * (best_index - 1)], 9 * 8);
					}

					for(int ppp = 0; ppp < best_index + 1; ppp++) {
						for(int pp = 0; pp < 5; pp++) {
							for(int pppp = 0; pppp < 5; pppp++) {
								CV_MAT_ELEM(*bestindex_plot, float, 5 * (params.N_iter + 1) - 1 - (ppp * 5 + pppp), par * 5 + pp) = 1;
							}
						}
					}


					for(int n_inner = params.N_c * par; n_inner < params.N_c * (par + 1); n_inner++) {
						for(int row = 0; row < 8; row++) {
							for(int col = 0; col < 8; col++) {
								Covariance[row][col] = New_Cov[row * 8 + col];
							}
						}
						dpotrf_(&UPLO, &order, &Covariance[0][0], &order, &info);
						for(int row = 0; row < 8; row++) {
							for(int col = row; col < 8; col++) {
								CV_MAT_ELEM(*CH, float, row, col) = (float)Covariance[row][col];
							}
						}

						cvRandArr(&rng_state, random, CV_RAND_NORMAL, cvScalar(0), cvScalar(1));
						cvGEMM(random, CH, 1, NULL, 0, colored_random);
						for(int num = 0; num < 8; num++) {
							scaled_rand[num] = (double)CV_MAT_ELEM(*colored_random, float, 0, num);
						}
						sl3(&scaled_rand[0], &sl3_rand[0]);
						mat_exp(&sl3_rand[0], &update_exp[0]);

						memcpy(&previous_X, &New_Mean[0], 9 * 8);
						mat_mul_3x3(&previous_X[0], &update_exp[0], &pred_X[0]);
						memcpy(&X_pred[n_inner * 9], &pred_X[0], 9 * 8);
						mat_inv_3x3(&X_par[par * 9], &inv_X[0]);
						mat_mul_3x3(&inv_X[0], &pred_X[0], &pred_AR[0]);
						memcpy(&AR_pred[n_inner * 9], &pred_AR[0], 9 * 8);

						imwarping_NN(img, &pred_X[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
						mean_v = cvAvg(warped_img);
						cvSubS(warped_img, mean_v, adj_warped_img);
						u = cvDotProduct(adj_warped_img, adj_template);
						v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
						NCC = u / v;

						cvSub(cv_mean_img, warped_img, diff_img);
						proj_sum = 0;
						for(int i = 0; i < num_used_basis; i++) {
							memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
							CV_MAT_ELEM(*projected, float, 0, i) = cvDotProduct(temp_basis, diff_img);
							proj_sum = proj_sum + CV_MAT_ELEM(*projected, float, 0, i) * CV_MAT_ELEM(*projected, float, 0, i);
						}
						PCA = cvDotProduct(diff_img, diff_img) - proj_sum;

						dist1 = -0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std - 0.5 * (PCA_real - PCA) * (PCA_real - PCA) / params.PCA_std / params.PCA_std;
						mat_inv_3x3(&Prev_Mean[0], &inv_X[0]);
						mat_mul_3x3(&inv_X[0], &pred_X[0], &pred_AR[0]);
						mat_log(&pred_AR[0], &log_AR[0]);
						sl3_to_vec(&log_AR[0], &X_diff[0]);
						for(int row = 0; row < 8; row++) {
							for(int col = 0; col < 8; col++) {
								CV_MAT_ELEM(*COV2, float, row, col) = (float)Prev_Cov[row * 8 + col];
							}
						}
						cvInvert(COV2, inv_COV2);
						for(int num = 0; num < 8; num++) {
							CV_MAT_ELEM(*X_diff_cv, float, 0, num) = (float)X_diff[num];
						}
						cvGEMM(X_diff_cv, inv_COV2, -0.5, NULL, 0, dist2_temp1);
						cvGEMM(dist2_temp1, X_diff_cv, 1, NULL, 0, dist2_temp2, CV_GEMM_B_T);
						dist2 = (double)CV_MAT_ELEM(*dist2_temp2, float, 0, 0);

						for(int row = 0; row < 8; row++) {
							for(int col = 0; col < 8; col++) {
								CV_MAT_ELEM(*COV3, float, row, col) = (float)New_Cov[row * 8 + col];
							}
						}
						cvInvert(COV3, inv_COV3);
						cvGEMM(colored_random, inv_COV3, -0.5, NULL, 0, dist3_temp1);
						cvGEMM(dist3_temp1, colored_random, 1, NULL, 0, dist3_temp2, CV_GEMM_B_T);
						dist3 = (double)CV_MAT_ELEM(*dist3_temp2, float, 0, 0);
						dist = MMAX(dist1 + dist2 - dist3, -250);
						prob[n_inner] = exp(dist) * (1 / ((float)cvSqrt((double)cvDet(COV2)))) / (1 / ((float)cvSqrt((double)cvDet(COV3))));
					}
					previous_index = outindex[par];
				} else { // if(params.outlier_flag == 0) 
					if(previous_index != outindex[par]) {
						num_est++;
						memcpy(&previous_X, &X_par[par * 9], 9 * 8);
						memcpy(&AR, &AR_velocity[par * 9], 9 * 8);

						mat_log(&AR[0], &log_AR[0]);
						mat_scalar(&log_AR[0], params.AR_p, &a_log_AR[0]);

						mat_exp(&a_log_AR[0], &update_exp[0]);
						mat_mul_3x3(&previous_X[0], &update_exp[0], &X_par_temp[0]);

						imwarping_NN(img, &X_par_temp[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
						mean_v = cvAvg(warped_img);
						cvSubS(warped_img, mean_v, adj_warped_img);

						cvSub(cv_mean_img, warped_img, diff_img);
						proj_sum = 0;
						cvSetZero(PCA_recon);
						for(int i = 0; i < num_used_basis; i++) {
							memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
							CV_MAT_ELEM(*projected, float, 0, i) = cvDotProduct(temp_basis, diff_img);
							proj_sum = proj_sum + CV_MAT_ELEM(*projected, float, 0, i) * CV_MAT_ELEM(*projected, float, 0, i);

							CV_MAT_ELEM(*PCA_recon_weight, float, 0, 0) = CV_MAT_ELEM(*projected, float, 0, i);
							cvGEMM(PCA_recon_weight, temp_basis, 1, NULL, 0, PCA_recon_temp);
							cvAdd(PCA_recon_temp, PCA_recon, PCA_recon);
						}
						PCA = cvDotProduct(diff_img, diff_img) - proj_sum;

						cvSub(diff_img, PCA_recon, recon_error);
						for(int num = 0; num < p_x * p_y; num++) {
							if(abs(CV_MAT_ELEM(*recon_error, float, 0, num)) > outlier_thr) {
								CV_MAT_ELEM(*adj_warped_img, float, 0, num) = 0;
							}
						}
						u = cvDotProduct(adj_warped_img, adj_template);
						v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
						NCC = u / v;

						memcpy(&Mean[0], &X_par_temp[0], 9 * 8);
						memcpy(&Cov[0], &state_cov[0], 64 * 8);

						diff1 = -0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std -
							0.5 * (PCA_real - PCA) * (PCA_real - PCA) / params.PCA_std / params.PCA_std;
						pred_measure[0] = exp(diff1);

						for(int iter = 1; iter < params.N_iter + 1; iter++) {
							cvGEMM(adj_warped_img, jacobian, 1 / v, NULL, 0, NCC_J_temp1);
							cvGEMM(adj_template, jacobian, (-u) / (v * v * v)*cvDotProduct(adj_warped_img, adj_warped_img), NULL, 0, NCC_J_temp2);
							cvAdd(NCC_J_temp1, NCC_J_temp2, NCC_jacobian);

							cvSub(diff_img, PCA_recon, d_PCA_measure);
							CV_MAT_ELEM(*PCA_recon_weight, float, 0, 0) = 1;
							cvGEMM(PCA_recon_weight, d_PCA_measure, 2, NULL, 0, d_PCA_measure);
							cvGEMM(d_PCA_measure, mean_jacobian, 1, NULL, 0, PCA_jacobian);

							for(int num = 0; num < 8; num++) {
								CV_MAT_ELEM(*total_jacobian, float, 0, num) = CV_MAT_ELEM(*NCC_jacobian, float, 0, num);
								CV_MAT_ELEM(*total_jacobian, float, 1, num) = CV_MAT_ELEM(*PCA_jacobian, float, 0, num);
							}

							t_mu_2[0] = NCC;
							t_mu_2[1] = PCA;
							for(int row = 0; row < 8; row++) {
								for(int col = 0; col < 8; col++) {
									CV_MAT_ELEM(*sigma_11, float, row, col) = Cov[64 * (iter - 1) + row * 8 + col];
								}
							}

							cvGEMM(sigma_11, total_jacobian, 1, NULL, 0, t_sigma_12, CV_GEMM_B_T);
							cvMatMul(total_jacobian, t_sigma_12, t_sigma_22);
							cvAdd(t_sigma_22, total_like, t_sigma_22);

							cvInvert(t_sigma_22, inv_t_sigma_22);
							cvMatMul(t_sigma_12, inv_t_sigma_22, t_temp_Cov1);
							cvGEMM(t_temp_Cov1, t_sigma_12, -1, sigma_11, 1, temp_Cov2, CV_GEMM_B_T);
							for(int row = 0; row < 8; row++) {
								for(int col = 0; col < 8; col++) {
									Cov[64 * iter + row * 8 + col] = CV_MAT_ELEM(*temp_Cov2, float, row, col);
								}
							}

							CvMat* temp_diff = cvCreateMat(2, 1, CV_32F);
							CV_MAT_ELEM(*temp_diff, float, 0, 0) = MMIN(NCC_real - t_mu_2[0], update_NCC_thr);
							CV_MAT_ELEM(*temp_diff, float, 1, 0) = MMAX(PCA_real - t_mu_2[1], update_PCA_thr);
							cvMatMul(t_temp_Cov1, temp_diff, temp_increment);
							for(int num = 0; num < 8; num++) {
								increment[num] = CV_MAT_ELEM(*temp_increment, float, num, 0);
							}

							sl3(&increment[0], &log_AR[0]);
							mat_exp(&log_AR[0], &update_exp[0]);
							mat_mul_3x3(&Mean[9 * (iter - 1)], &update_exp[0], &Mean[9 * iter]);

							imwarping_NN(img, &Mean[9 * iter], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
							mean_v = cvAvg(warped_img);
							cvSubS(warped_img, mean_v, adj_warped_img);

							cvSub(cv_mean_img, warped_img, diff_img);
							proj_sum = 0;
							cvSetZero(PCA_recon);
							for(int i = 0; i < num_used_basis; i++) {
								memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
								CV_MAT_ELEM(*projected, float, 0, i) = cvDotProduct(temp_basis, diff_img);
								proj_sum = proj_sum + CV_MAT_ELEM(*projected, float, 0, i) * CV_MAT_ELEM(*projected, float, 0, i);

								CV_MAT_ELEM(*PCA_recon_weight, float, 0, 0) = CV_MAT_ELEM(*projected, float, 0, i);
								cvGEMM(PCA_recon_weight, temp_basis, 1, NULL, 0, PCA_recon_temp);
								cvAdd(PCA_recon_temp, PCA_recon, PCA_recon);
							}
							PCA = cvDotProduct(diff_img, diff_img) - proj_sum;

							cvSub(diff_img, PCA_recon, recon_error);
							for(int num = 0; num < p_x * p_y; num++) {
								if(abs(CV_MAT_ELEM(*recon_error, float, 0, num)) > outlier_thr) {
									CV_MAT_ELEM(*adj_warped_img, float, 0, num) = 0;
								}
							}
							u = cvDotProduct(adj_warped_img, adj_template);
							v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
							NCC = u / v;

							diff1 = -0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std -
								0.5 * (PCA_real - PCA) * (PCA_real - PCA) / params.PCA_std / params.PCA_std;
							cvInvert(sigma_11, inv_COV2);
							cvGEMM(temp_increment, inv_COV2, -0.5, NULL, 0, dist2_temp1, CV_GEMM_A_T);
							cvGEMM(dist2_temp1, temp_increment, 1, NULL, 0, dist2_temp2);
							diff2 = (double)CV_MAT_ELEM(*dist2_temp2, float, 0, 0);
							pred_measure[iter] = exp(diff1 + diff2);
						}
						memcpy((double*)(temp_pred_measure->data.ptr), &pred_measure[0], (params.N_iter + 1) * 8);
						cvMinMaxLoc(temp_pred_measure, &min_v, &max_v, &min_ind, &max_ind);
						best_index = max_ind.x;
					} // if(previous_index != outindex[par]) 
					memcpy(&New_Cov[0], &Cov[64 * best_index], 64 * 8);
					memcpy(&New_Mean[0], &Mean[9 * best_index], 9 * 8);
					if(best_index == 0) {
						memcpy(&Prev_Cov[0], &Cov[64 * best_index], 64 * 8);
						memcpy(&Prev_Mean[0], &Mean[9 * best_index], 9 * 8);
					} else {
						memcpy(&Prev_Cov[0], &Cov[64 * (best_index - 1)], 64 * 8);
						memcpy(&Prev_Mean[0], &Mean[9 * (best_index - 1)], 9 * 8);
					}
					for(int ppp = 0; ppp < best_index + 1; ppp++) {
						for(int pp = 0; pp < 5; pp++) {
							for(int pppp = 0; pppp < 5; pppp++) {
								CV_MAT_ELEM(*bestindex_plot, float, 5 * (params.N_iter + 1) - 1 - (ppp * 5 + pppp), par * 5 + pp) = 1;
							}
						}
					}

					for(int n_inner = params.N_c * par; n_inner < params.N_c * (par + 1); n_inner++) {
						for(int row = 0; row < 8; row++) {
							for(int col = 0; col < 8; col++) {
								Covariance[row][col] = New_Cov[row * 8 + col];
							}
						}
						dpotrf_(&UPLO, &order, &Covariance[0][0], &order, &info);
						for(int row = 0; row < 8; row++) {
							for(int col = row; col < 8; col++) {
								CV_MAT_ELEM(*CH, float, row, col) = (float)Covariance[row][col];
							}
						}

						cvRandArr(&rng_state, random, CV_RAND_NORMAL, cvScalar(0), cvScalar(1));
						cvGEMM(random, CH, 1, NULL, 0, colored_random);
						for(int num = 0; num < 8; num++) {
							scaled_rand[num] = (double)CV_MAT_ELEM(*colored_random, float, 0, num);
						}
						sl3(&scaled_rand[0], &sl3_rand[0]);
						mat_exp(&sl3_rand[0], &update_exp[0]);

						memcpy(&previous_X, &New_Mean[0], 9 * 8);
						mat_mul_3x3(&previous_X[0], &update_exp[0], &pred_X[0]);
						memcpy(&X_pred[n_inner * 9], &pred_X[0], 9 * 8);
						mat_inv_3x3(&X_par[par * 9], &inv_X[0]);
						mat_mul_3x3(&inv_X[0], &pred_X[0], &pred_AR[0]);
						memcpy(&AR_pred[n_inner * 9], &pred_AR[0], 9 * 8);

						imwarping_NN(img, &pred_X[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
						mean_v = cvAvg(warped_img);
						cvSubS(warped_img, mean_v, adj_warped_img);

						cvSub(cv_mean_img, warped_img, diff_img);
						proj_sum = 0;
						cvSetZero(PCA_recon);
						for(int i = 0; i < num_used_basis; i++) {
							memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
							CV_MAT_ELEM(*projected, float, 0, i) = cvDotProduct(temp_basis, diff_img);
							proj_sum = proj_sum + CV_MAT_ELEM(*projected, float, 0, i) * CV_MAT_ELEM(*projected, float, 0, i);

							CV_MAT_ELEM(*PCA_recon_weight, float, 0, 0) = CV_MAT_ELEM(*projected, float, 0, i);
							cvGEMM(PCA_recon_weight, temp_basis, 1, NULL, 0, PCA_recon_temp);
							cvAdd(PCA_recon_temp, PCA_recon, PCA_recon);
						}
						PCA = cvDotProduct(diff_img, diff_img) - proj_sum;

						cvSub(diff_img, PCA_recon, recon_error);
						for(int num = 0; num < p_x * p_y; num++) {
							if(abs(CV_MAT_ELEM(*recon_error, float, 0, num)) > outlier_thr) {
								CV_MAT_ELEM(*adj_warped_img, float, 0, num) = 0;
							}
						}
						u = cvDotProduct(adj_warped_img, adj_template);
						v = sqrt(cvDotProduct(adj_warped_img, adj_warped_img) * cvDotProduct(adj_template, adj_template));
						NCC = u / v;

						dist1 = -0.5 * (NCC_real - NCC) * (NCC_real - NCC) / params.NCC_std / params.NCC_std -
							0.5 * (PCA_real - PCA) * (PCA_real - PCA) / params.PCA_std / params.PCA_std;
						mat_inv_3x3(&Prev_Mean[0], &inv_X[0]);
						mat_mul_3x3(&inv_X[0], &pred_X[0], &pred_AR[0]);
						mat_log(&pred_AR[0], &log_AR[0]);
						sl3_to_vec(&log_AR[0], &X_diff[0]);
						for(int row = 0; row < 8; row++) {
							for(int col = 0; col < 8; col++) {
								CV_MAT_ELEM(*COV2, float, row, col) = (float)Prev_Cov[row * 8 + col];
							}
						}
						cvInvert(COV2, inv_COV2);
						for(int num = 0; num < 8; num++) {
							CV_MAT_ELEM(*X_diff_cv, float, 0, num) = (float)X_diff[num];
						}
						cvGEMM(X_diff_cv, inv_COV2, -0.5, NULL, 0, dist2_temp1);
						cvGEMM(dist2_temp1, X_diff_cv, 1, NULL, 0, dist2_temp2, CV_GEMM_B_T);
						dist2 = (double)CV_MAT_ELEM(*dist2_temp2, float, 0, 0);

						for(int row = 0; row < 8; row++) {
							for(int col = 0; col < 8; col++) {
								CV_MAT_ELEM(*COV3, float, row, col) = (float)New_Cov[row * 8 + col];
							}
						}
						cvInvert(COV3, inv_COV3);
						cvGEMM(colored_random, inv_COV3, -0.5, NULL, 0, dist3_temp1);
						cvGEMM(dist3_temp1, colored_random, 1, NULL, 0, dist3_temp2, CV_GEMM_B_T);
						dist3 = (double)CV_MAT_ELEM(*dist3_temp2, float, 0, 0);
						dist = MMAX(dist1 + dist2 - dist3, -250);
						prob[n_inner] = exp(dist) * (1 / ((float)cvSqrt((double)cvDet(COV2)))) / (1 / ((float)cvSqrt((double)cvDet(COV3))));
					} // for(int n_inner = params.N_c * par; n_inner < params.N_c * (par + 1); n_inner++)
					previous_index = outindex[par];
				}
			}
		}
	}
	num_est = 0;
	prob_sum = elem_sum(prob, params.N * params.N_c);
	elem_div(prob, prob_sum, w, params.N * params.N_c);
	resampling(w, params.N * params.N_c, params.N, outindex);
	particle_copy(outindex, params.N, X_pred, AR_pred, X_par, AR_velocity);
	memcpy((double*)(temp_w->data.ptr), w, params.N * params.N_c * 8);
	cvMinMaxLoc(temp_w, &min_v, &max_v, &min_ind, &max_ind);
	memcpy(&max_X[0], &X_pred[9 * max_ind.x], 9 * 8);
	resampling(w, params.N * params.N_c, params.N * params.N_c, outindex2);
	particle_copy(outindex2, params.N * params.N_c, X_pred, AR_pred, X_par_mean, AR_velocity_mean);
	sample_mean(X_par_mean, &max_X[0], params.N * params.N_c, &mean_X_par[0]);

	if(params.show_weights){
		//Display particle weights
		cvSetZero(weight_plot);
		for(int num = 0; num < params.N * params.N_c; num++) {
			int num_pixel;
			num_pixel = ceil(w[num] / max_v * 50);
			for(int pixel = 49; pixel > 50 - num_pixel; pixel--) {
				CV_MAT_ELEM(*weight_plot, float, pixel, num) = 1;
			}
		}
	}

	// Extract the tracked result
	imwarping_NN(img, &mean_X_par[0], p_matrix, &p_transform[0], c_x, c_y, p_x, p_y, frame_size, warped_img);
	warped_img_2D = cvReshape(warped_img, &warped_img_2D_header, 0, p_x);
	memcpy(&tracked_img[p_x * p_y * frame_id], (float*)(warped_img->data.ptr), p_x * p_y * 4);
	if(params.show_templates) {
		for(int row = 0; row < p_x; row++) {
			for(int col = 0; col < p_y; col++) {
				CV_MAT_ELEM(*templ, float, row, col) = CV_MAT_ELEM(*warped_img_2D, float, row, col);
			}
		}
	}

	// Select update images that contain few outliers
	if(PCA_flag == 1) {
		if(params.outlier_flag == 0) {
			memcpy(&update_img[p_x * p_y * num_update_img], &tracked_img[p_x * p_y * frame_id], p_x * p_y * 4);
			num_update_img = num_update_img + 1;
			num_tracked_img = num_tracked_img + 1;
		} else {
			cvSub(cv_mean_img, warped_img, diff_img);
			proj_sum = 0;
			cvSetZero(PCA_recon);
			for(int i = 0; i < num_used_basis; i++) {
				memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
				CV_MAT_ELEM(*projected, float, 0, i) = cvDotProduct(temp_basis, diff_img);
				proj_sum = proj_sum + CV_MAT_ELEM(*projected, float, 0, i) * CV_MAT_ELEM(*projected, float, 0, i);
				CV_MAT_ELEM(*PCA_recon_weight, float, 0, 0) = CV_MAT_ELEM(*projected, float, 0, i);
				cvGEMM(PCA_recon_weight, temp_basis, 1, NULL, 0, PCA_recon_temp);
				cvAdd(PCA_recon_temp, PCA_recon, PCA_recon);
			}
			cvSub(diff_img, PCA_recon, recon_error);
			for(int num = 0; num < p_x * p_y; num++) {
				if(abs(CV_MAT_ELEM(*recon_error, float, 0, num)) > outlier_thr) {
					num_outlier++;
				}
			}
			if(num_outlier < update_thr * p_x * p_y) {
				memcpy(&update_img[p_x * p_y * num_update_img], &tracked_img[p_x * p_y * frame_id], p_x * p_y * 4);
				num_update_img = num_update_img + 1;
				num_tracked_img = num_tracked_img + 1;
			}
			num_outlier = 0;
		}
	} else {
		num_tracked_img = num_tracked_img + 1;
	}

	// Perform Incremetnal PCA
	if(num_tracked_img >= params.init_size && PCA_flag == 0) {
		cal_mean_img(&tracked_img[0], p_x * p_y, params.init_size, &mean_img[0]);
		memcpy((float*)(cv_mean_img->data.ptr), &mean_img[0], p_x * p_y * 4);
		cal_mean_adj_img(&tracked_img[0], &mean_img[0], p_x * p_y, params.init_size, &mean_adj_img[0]);

		for(int i = 0; i < params.init_size; i++) {
			memcpy((float*)(A->data.ptr + A->step * i), &mean_adj_img[p_x * p_y * i], p_x * p_y * 4);
		}
		cvSVD(A, W, U, V, CV_SVD_V_T);
		double index = find_index(W, params.basis_thr);
		num_basis = MMIN(params.max_num_basis, index);
		num_used_basis = MMIN(num_basis, params.max_num_used_basis);
		for(int i = 0; i < num_basis; i++) {
			memcpy((float*)(basis->data.ptr + basis->step * i), (float*)(V->data.ptr + V->step * i), p_x * p_y * 4);
			CV_MAT_ELEM(*Sing_val, float, 0, i) = CV_MAT_ELEM(*W, float, i, i);

			if(i < num_used_basis) {
				memcpy((float*)(warped_img->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
				warped_img_2D = cvReshape(warped_img, &warped_img_2D_header, 0, p_x);
				cvMinMaxLoc(warped_img_2D, &min_v, &max_v);
				cvSubS(warped_img_2D, cvScalar(min_v), warped_img_2D);
				cvSetZero(divide);
				cvAddS(divide, cvScalar(max_v - min_v), divide);
				cvDiv(warped_img_2D, divide, warped_img_2D);
				if(params.show_templates) {
					for(int row = 0; row < p_x; row++) {
						for(int col = 0; col < p_y; col++) {
							CV_MAT_ELEM(*templ, float, row, col + (i + 3)*p_y) = CV_MAT_ELEM(*warped_img_2D, float, row, col);
						}
					}
				}
			}
		}
		warped_img_2D = cvReshape(cv_mean_img, &warped_img_2D_header, 0, p_x);
		if(params.show_templates) {
			for(int row = 0; row < p_x; row++) {
				for(int col = 0; col < p_y; col++) {
					CV_MAT_ELEM(*templ, float, row, col + p_y + p_y) = CV_MAT_ELEM(*warped_img_2D, float, row, col);
				}
			}
		}

		//cout << "Updated PCA basis at " << t <<" frame, " << num_basis << " PC's from " << num_tracked_img << " images"<< endl;
		PCA_flag = 1;

		// Mean gradient calculation
		warped_img_2D = cvReshape(cv_mean_img, &warped_img_2D_header, 0, p_x);
		image_gradient(warped_img_2D, p_x, p_y, grad_x, grad_y);

		// Mean jacobian calculation
		jacobian_calculation(p_matrix, grad_x, grad_y, dX_du, p_x, p_y, mean_jacobian);
	} else if(num_update_img == params.update_period && PCA_flag == 1) {
		cal_mean_img(&update_img[0], p_x * p_y, params.update_period, &mean_update_img[0]);
		cal_mean_adj_img(&update_img[0], &mean_update_img[0], p_x * p_y, params.update_period, &mean_adj_img[0]);
		memcpy(&new_data[0], &mean_adj_img[0], p_x * p_y * params.update_period * 4);
		weight = cvSqrt(((float)(num_tracked_img - params.update_period)) * params.update_period / ((float)(num_tracked_img)));
		img_diff_scalar_mul(&mean_update_img[0], &mean_img[0], weight, p_x * p_y, &temp_img[0]);
		memcpy(&new_data[p_x * p_y * params.update_period], &temp_img[0], p_x * p_y * 4);
		mean_update(&mean_img[0], &mean_update_img[0], params.ff, params.update_period, num_tracked_img, p_x * p_y);
		memcpy((float*)(cv_mean_img->data.ptr), &mean_img[0], p_x * p_y * 4);

		for(int i = 0; i < num_basis; i++) {
			memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
			for(int j = 0; j < params.update_period + 1; j++) {
				memcpy((float*)(diff_img->data.ptr), &new_data[p_x * p_y * j], p_x * p_y * 4);
				CV_MAT_ELEM(*data_proj, float, j, i) = cvDotProduct(temp_basis, diff_img);
			}
		}

		for(int j = 0; j < params.update_period + 1; j++) {
			cvSetZero(PCA_recon);
			for(int i = 0; i < num_basis; i++) {
				memcpy((float*)(temp_basis->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);

				CvMat* temp_recon = cvCreateMat(1, p_x * p_y, CV_32F);
				CvMat* temp_proj = cvCreateMat(1, p_x * p_y, CV_32F);
				cvSetZero(temp_proj);
				cvAddS(temp_proj, cvScalar(CV_MAT_ELEM(*data_proj, float, j, i)), temp_proj);
				cvMul(temp_basis, temp_proj, temp_recon, 1);
				cvAdd(PCA_recon, temp_recon, PCA_recon);

				cvReleaseMat(&temp_recon);
				cvReleaseMat(&temp_proj);
			}
			memcpy((float*)(diff_img->data.ptr), &new_data[p_x * p_y * j], p_x * p_y * 4);
			cvSub(diff_img, PCA_recon, diff_img);
			memcpy((float*)(data_res->data.ptr + data_res->step * j), (float*)(diff_img->data.ptr), p_x * p_y * 4);
		}

		for(int num = 0; num < params.update_period + 1; num++) {
			for(int pixel = 0; pixel < p_x * p_y; pixel++) {
				temp_res[pixel + num * (p_x * p_y)] = (double)CV_MAT_ELEM(*data_res, float, num, pixel);
			}
		}
		dgeqrf_(&qr_M, &qr_N, &temp_res[0], &qr_LDA, tau, WORK, &qr_LWORK, &qr_INFO);
		dorgqr_(&qr_M, &qr_N, &qr_N, &temp_res[0], &qr_LDA, tau, WORK, &qr_LWORK, &qr_INFO);

		for(int num = 0; num < params.update_period + 1; num++) {
			for(int pixel = 0; pixel < p_x * p_y; pixel++) {
				CV_MAT_ELEM(*QR_Q, float, num, pixel) = (float)temp_res[pixel + num * (p_x * p_y)];
			}
		}

		CvMat* Total_Q = cvCreateMat(num_basis + params.update_period + 1, p_x * p_y, CV_32F);
		for(int num = 0; num < num_basis; num++) {
			memcpy((float*)(Total_Q->data.ptr + Total_Q->step * num), (float*)(basis->data.ptr + basis->step * num), p_x * p_y * 4);
		}
		for(int num = 0; num < params.update_period + 1; num++) {
			memcpy((float*)(Total_Q->data.ptr + Total_Q->step * (num + num_basis)), (float*)(QR_Q->data.ptr + QR_Q->step * num), p_x * p_y * 4);
		}

		CvMat* Total_R = cvCreateMat(num_basis + params.update_period + 1, num_basis + params.update_period + 1, CV_32F);
		cvSetZero(Total_R);
		for(int num = 0; num < num_basis; num++) {
			CV_MAT_ELEM(*Total_R, float, num, num) = CV_MAT_ELEM(*Sing_val, float, 0, num) * params.ff;
		}
		for(int num = 0; num < params.update_period + 1; num++) {
			for(int bb = 0; bb < num_basis; bb++) {
				CV_MAT_ELEM(*Total_R, float, num + num_basis, bb) = CV_MAT_ELEM(*data_proj, float, num, bb);
			}
		}
		cvGEMM(data_res, QR_Q, 1, NULL, 0, Q_proj, CV_GEMM_B_T);
		for(int num = 0; num < params.update_period + 1; num++) {
			for(int bb = 0; bb < params.update_period + 1; bb++) {
				CV_MAT_ELEM(*Total_R, float, num + num_basis, bb + num_basis) = CV_MAT_ELEM(*Q_proj, float, num, bb);
			}
		}

		CvMat* PCA_U = cvCreateMat(num_basis + params.update_period + 1, num_basis + params.update_period + 1, CV_32F);
		CvMat* PCA_D = cvCreateMat(num_basis + params.update_period + 1, num_basis + params.update_period + 1, CV_32F);
		CvMat* PCA_V = cvCreateMat(num_basis + params.update_period + 1, num_basis + params.update_period + 1, CV_32F);
		cvSVD(Total_R, PCA_D, PCA_U, PCA_V, CV_SVD_V_T);
		CvMat* temp_total_basis = cvCreateMat(num_basis + params.update_period + 1, p_x * p_y, CV_32F);
		cvMatMul(PCA_V, Total_Q, temp_total_basis);

		double index = find_index(PCA_D, params.basis_thr);
		num_basis = MMIN(params.max_num_basis, index);
		num_used_basis = MMIN(num_basis, params.max_num_used_basis);

		for(int i = 0; i < num_basis; i++) {
			memcpy((float*)(basis->data.ptr + basis->step * i), (float*)(temp_total_basis->data.ptr + temp_total_basis->step * i), p_x * p_y * 4);
			CV_MAT_ELEM(*Sing_val, float, 0, i) = CV_MAT_ELEM(*PCA_D, float, i, i);

			if(i < num_used_basis) {
				memcpy((float*)(warped_img->data.ptr), (float*)(basis->data.ptr + basis->step * i), p_x * p_y * 4);
				warped_img_2D = cvReshape(warped_img, &warped_img_2D_header, 0, p_x);
				cvMinMaxLoc(warped_img_2D, &min_v, &max_v);
				cvSubS(warped_img_2D, cvScalar(min_v), warped_img_2D);
				cvSetZero(divide);
				cvAddS(divide, cvScalar(max_v - min_v), divide);
				cvDiv(warped_img_2D, divide, warped_img_2D);
				if(params.show_templates) {
					for(int row = 0; row < p_x; row++) {
						for(int col = 0; col < p_y; col++) {
							CV_MAT_ELEM(*templ, float, row, col + (i + 3)*p_y) = CV_MAT_ELEM(*warped_img_2D, float, row, col);
						}
					}
				}
			}
		}
		warped_img_2D = cvReshape(cv_mean_img, &warped_img_2D_header, 0, p_x);
		if(params.show_templates) {
			for(int row = 0; row < p_x; row++) {
				for(int col = 0; col < p_y; col++) {
					CV_MAT_ELEM(*templ, float, row, col + p_y + p_y) = CV_MAT_ELEM(*warped_img_2D, float, row, col);
				}
			}
		}

		//cout << "Updated PCA basis at " << t <<" frame, " << num_basis << " PC's from " << num_tracked_img << " images"<< endl;
		num_update_img = 0;

		// Mean gradient calculation
		warped_img_2D = cvReshape(cv_mean_img, &warped_img_2D_header, 0, p_x);
		image_gradient(warped_img_2D, p_x, p_y, grad_x, grad_y);

		// Mean jacobian calculation
		jacobian_calculation(p_matrix, grad_x, grad_y, dX_du, p_x, p_y, mean_jacobian);

		cvReleaseMat(&Total_Q);
		cvReleaseMat(&Total_R);
		cvReleaseMat(&PCA_U);
		cvReleaseMat(&PCA_D);
		cvReleaseMat(&PCA_V);
		cvReleaseMat(&temp_total_basis);
	}
	if(params.show_templates) {
		cvShowImage("Templates", templ);
		cvWaitKey(1);
	}
	if(params.show_weights) {
		cvShowImage("Particle Weights", weight_plot);
		cvWaitKey(1);
	}

	if(params.sampling) {
		if(params.show_weights) {
			cvShowImage("Best index", bestindex_plot);
			cvWaitKey(1);
			cvSetZero(bestindex_plot);
		}
	}
	free(temp_res);
	free(tau);
	free(WORK);

	updateCVCorners();

}
//Image warping using nearest neighbor
void PFSL3::PFSL3::imwarping_NN(CvMat* img, double* h_matrix, double* p_matrix, 
	double* p_transform, int c_x, int c_y, int p_x, int p_y, CvSize frame_size, 
	CvMat* warped_img) {
	double px, py, pz, npx, npy, nnpx, nnpy;
	int tpx, tpy;

	for(int i = 0; i < p_x * p_y; i++) {
		px = h_matrix[0] * p_matrix[0 + i * 2] + h_matrix[1] * p_matrix[1 + i * 2] + h_matrix[2];
		py = h_matrix[3] * p_matrix[0 + i * 2] + h_matrix[4] * p_matrix[1 + i * 2] + h_matrix[5];
		pz = h_matrix[6] * p_matrix[0 + i * 2] + h_matrix[7] * p_matrix[1 + i * 2] + h_matrix[8];
		npx = px / pz;
		npy = py / pz;
		nnpx = MMIN(p_transform[0] * npx + p_transform[2] + c_x - 1, frame_size.height - 1);
		nnpy = MMIN(p_transform[4] * npy + p_transform[5] + c_y - 1, frame_size.width - 1);
		tpx = MMAX(MMIN((int)(nnpx + 0.5f), frame_size.height), 0);
		tpy = MMAX(MMIN((int)(nnpy + 0.5f), frame_size.width), 0);
		CV_MAT_ELEM(*warped_img, float, 0, i) = (CV_MAT_ELEM(*img, float, tpx, tpy)) / 255;
	}
}

//Image warping using bilinear
void PFSL3::imwarping_BL(CvMat* img, double* h_matrix, double* p_matrix, 
	double* p_transform, int c_x, int c_y, int p_x, int p_y, 
	CvSize frame_size, CvMat* warped_img) {
	double px, py, pz, npx, npy, nnpx, nnpy, floor_x, ceil_x, floor_y, ceil_y, p1, p2, p3, p4;

	for(int i = 0; i < p_x * p_y; i++) {
		px = h_matrix[0] * p_matrix[0 + i * 2] + h_matrix[1] * p_matrix[1 + i * 2] + h_matrix[2];
		py = h_matrix[3] * p_matrix[0 + i * 2] + h_matrix[4] * p_matrix[1 + i * 2] + h_matrix[5];
		pz = h_matrix[6] * p_matrix[0 + i * 2] + h_matrix[7] * p_matrix[1 + i * 2] + h_matrix[8];
		npx = px / pz;
		npy = py / pz;
		nnpx = MMIN(p_transform[0] * npx + p_transform[2] + c_x - 1, frame_size.height - 1);
		nnpy = MMIN(p_transform[4] * npy + p_transform[5] + c_y - 1, frame_size.width - 1);

		floor_x = floor(MMAX(nnpx, 0));
		ceil_x = MMIN(floor_x + 1, frame_size.height - 1);
		floor_y = floor(MMAX(nnpy, 0));
		ceil_y = MMIN(floor_y + 1, frame_size.width - 1);

		p1 = ((double)CV_MAT_ELEM(*img, float, (int)floor_x, (int)floor_y));
		p2 = ((double)CV_MAT_ELEM(*img, float, (int)floor_x, (int)ceil_y));
		p3 = ((double)CV_MAT_ELEM(*img, float, (int)ceil_x, (int)floor_y));
		p4 = ((double)CV_MAT_ELEM(*img, float, (int)ceil_x, (int)ceil_y));

		CV_MAT_ELEM(*warped_img, float, 0, i) = ((ceil_x - nnpx) * (ceil_y - nnpy) * p1 + (ceil_x - nnpx) * (nnpy - floor_y) * p2 + (nnpx - floor_x) * (ceil_y - nnpy) * p3 + (nnpx - floor_x) * (nnpy - floor_y) * p4) / 255;
	}
}


// Approximated matrix exponential
void PFSL3::mat_exp(double* input, double* output) {
	double F[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, C[9], E[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, A[9];

	memcpy(&A[0], input, 9 * 8);

	for(int i = 1; i < 12; i++) {
		for(int j = 0; j < 9; j++) {
			E[j] = E[j] + F[j];
		}

		C[0] = A[0] * F[0] + A[1] * F[3] + A[2] * F[6];
		C[1] = A[0] * F[1] + A[1] * F[4] + A[2] * F[7];
		C[2] = A[0] * F[2] + A[1] * F[5] + A[2] * F[8];
		C[3] = A[3] * F[0] + A[4] * F[3] + A[5] * F[6];
		C[4] = A[3] * F[1] + A[4] * F[4] + A[5] * F[7];
		C[5] = A[3] * F[2] + A[4] * F[5] + A[5] * F[8];
		C[6] = A[6] * F[0] + A[7] * F[3] + A[8] * F[6];
		C[7] = A[6] * F[1] + A[7] * F[4] + A[8] * F[7];
		C[8] = A[6] * F[2] + A[7] * F[5] + A[8] * F[8];

		for(int j = 0; j < 9; j++) {
			F[j] = C[j] / i;
		}
	}

	memcpy(output, &E[0], 9 * 8);
}

// Approximated matrix log
void PFSL3::mat_log(double* input, double* output) {
	double B[9], BB[9], C[9], L[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	memcpy(&B, input, 9 * 8);
	B[0] = B[0] - 1;
	B[4] = B[4] - 1;
	B[8] = B[8] - 1;

	memcpy(&BB, &B, 9 * 8);

	for(int i = 1; i < 12; i++) {
		for(int j = 0; j < 9; j++) {
			L[j] = L[j] + pow((double)-1, i + 1) * BB[j] / i;
		}

		C[0] = BB[0] * B[0] + BB[1] * B[3] + BB[2] * B[6];
		C[1] = BB[0] * B[1] + BB[1] * B[4] + BB[2] * B[7];
		C[2] = BB[0] * B[2] + BB[1] * B[5] + BB[2] * B[8];
		C[3] = BB[3] * B[0] + BB[4] * B[3] + BB[5] * B[6];
		C[4] = BB[3] * B[1] + BB[4] * B[4] + BB[5] * B[7];
		C[5] = BB[3] * B[2] + BB[4] * B[5] + BB[5] * B[8];
		C[6] = BB[6] * B[0] + BB[7] * B[3] + BB[8] * B[6];
		C[7] = BB[6] * B[1] + BB[7] * B[4] + BB[8] * B[7];
		C[8] = BB[6] * B[2] + BB[7] * B[5] + BB[8] * B[8];

		for(int j = 0; j < 9; j++) {
			BB[j] = C[j];
		}
	}

	memcpy(output, &L, 9 * 8);
}

void PFSL3::mat_mul_3x3(double* input_A, double* input_B, double* output) {
	output[0] = input_A[0] * input_B[0] + input_A[1] * input_B[3] + input_A[2] * input_B[6];
	output[1] = input_A[0] * input_B[1] + input_A[1] * input_B[4] + input_A[2] * input_B[7];
	output[2] = input_A[0] * input_B[2] + input_A[1] * input_B[5] + input_A[2] * input_B[8];
	output[3] = input_A[3] * input_B[0] + input_A[4] * input_B[3] + input_A[5] * input_B[6];
	output[4] = input_A[3] * input_B[1] + input_A[4] * input_B[4] + input_A[5] * input_B[7];
	output[5] = input_A[3] * input_B[2] + input_A[4] * input_B[5] + input_A[5] * input_B[8];
	output[6] = input_A[6] * input_B[0] + input_A[7] * input_B[3] + input_A[8] * input_B[6];
	output[7] = input_A[6] * input_B[1] + input_A[7] * input_B[4] + input_A[8] * input_B[7];
	output[8] = input_A[6] * input_B[2] + input_A[7] * input_B[5] + input_A[8] * input_B[8];
}

void PFSL3::sl3_to_vec(double* input, double* output) {
	output[0] = input[2];
	output[1] = input[5];
	output[2] = (input[3] - input[1]) / 2;
	output[3] = (input[3] + input[1]) / 2;
	output[4] = input[0];
	output[5] = input[8];
	output[6] = input[6];
	output[7] = input[7];
}

void PFSL3::sl3(double* input, double* output) {
	output[0] = input[4];
	output[1] = -input[2] + input[3];
	output[2] = input[0];
	output[3] = input[2] + input[3];
	output[4] = -input[4] - input[5];
	output[5] = input[1];
	output[6] = input[6];
	output[7] = input[7];
	output[8] = input[5];
}

void PFSL3::mat_scalar(double* input, double scalar, double* output) {
	output[0] = input[0] * scalar;
	output[1] = input[1] * scalar;
	output[2] = input[2] * scalar;
	output[3] = input[3] * scalar;
	output[4] = input[4] * scalar;
	output[5] = input[5] * scalar;
	output[6] = input[6] * scalar;
	output[7] = input[7] * scalar;
	output[8] = input[8] * scalar;
}

void PFSL3::mat_add(double* input_A, double* input_B, double* output) {
	output[0] = input_A[0] + input_B[0];
	output[1] = input_A[1] + input_B[1];
	output[2] = input_A[2] + input_B[2];
	output[3] = input_A[3] + input_B[3];
	output[4] = input_A[4] + input_B[4];
	output[5] = input_A[5] + input_B[5];
	output[6] = input_A[6] + input_B[6];
	output[7] = input_A[7] + input_B[7];
	output[8] = input_A[8] + input_B[8];
}

void PFSL3::mat_mul_elem(double* input_A, double* input_B, double* output) {
	output[0] = input_A[0] * input_B[0];
	output[1] = input_A[1] * input_B[1];
	output[2] = input_A[2] * input_B[2];
	output[3] = input_A[3] * input_B[3];
	output[4] = input_A[4] * input_B[4];
	output[5] = input_A[5] * input_B[5];
	output[6] = input_A[6] * input_B[6];
	output[7] = input_A[7] * input_B[7];
}

void PFSL3::mat_display(double* input, int size) {
	if(size == 9) {
		std::cout << input[0] << " " << input[1] << " " << input[2] << std::endl;
		std::cout << input[3] << " " << input[4] << " " << input[5] << std::endl;
		std::cout << input[6] << " " << input[7] << " " << input[8] << std::endl;
	} else {
		std::cout << input[0] << " " << input[1] << " " << input[2] << std::endl;
		std::cout << input[3] << " " << input[4] << " " << input[5] << std::endl;
		std::cout << input[6] << " " << input[7] << std::endl;
	}

}

// Particle resampling according to the weights
void PFSL3::resampling(double* w, int N, int M, int* outindex) {
	CvRNG rng_state = cvRNG(GetRDTSC());
	CvMat* U = cvCreateMat(1, 1, CV_64F);
	cvRandArr(&rng_state, U, CV_RAND_UNI, cvScalar(0), cvScalar(1));
	double UU = CV_MAT_ELEM(*U, double, 0, 0) / ((double)M);

	int k = 0;
	double* n_R;
	n_R = new double[N];
	for(int i = 0; i < N; i++) {
		n_R[i] = floor((w[i] - UU) * M) + 1;
		UU = UU + n_R[i] / M - w[i];

		if(n_R[i] != 0) {
			for(int j = 0; j < n_R[i]; j++) {
				outindex[k] = i;
				k++;
			}
		}
	}
	cvReleaseMat(&U);
}

// Particle copy with outindex
void PFSL3::particle_copy(int* outindex, int N, double* input_A, double* input_B, double* output_A, double* output_B) {
	int index;
	for(int i = 0; i < N; i++) {
		index = outindex[i];
		memcpy(&output_A[i * 9], &input_A[index * 9], 9 * 8);
		memcpy(&output_B[i * 9], &input_B[index * 9], 9 * 8);
	}
}

// Corner point transformatino for display
void PFSL3::point_transform(const CvMat* point, const double* h_matrix, const double* p_transform,
	int c_x, int c_y, CvMat* t_point) {
	double px, py, pz, npx, npy;

	px = h_matrix[0] * CV_MAT_ELEM(*point, double, 0, 0) + h_matrix[1] * CV_MAT_ELEM(*point, double, 1, 0) + h_matrix[2] * CV_MAT_ELEM(*point, double, 2, 0);
	py = h_matrix[3] * CV_MAT_ELEM(*point, double, 0, 0) + h_matrix[4] * CV_MAT_ELEM(*point, double, 1, 0) + h_matrix[5] * CV_MAT_ELEM(*point, double, 2, 0);
	pz = h_matrix[6] * CV_MAT_ELEM(*point, double, 0, 0) + h_matrix[7] * CV_MAT_ELEM(*point, double, 1, 0) + h_matrix[8] * CV_MAT_ELEM(*point, double, 2, 0);
	npx = px / pz;
	npy = py / pz;
	CV_MAT_ELEM(*t_point, double, 0, 0) = p_transform[0] * npx + p_transform[2] + c_x - 1;
	CV_MAT_ELEM(*t_point, double, 0, 1) = p_transform[4] * npy + p_transform[5] + c_y - 1;

	px = h_matrix[0] * CV_MAT_ELEM(*point, double, 0, 1) + h_matrix[1] * CV_MAT_ELEM(*point, double, 1, 1) + h_matrix[2] * CV_MAT_ELEM(*point, double, 2, 1);
	py = h_matrix[3] * CV_MAT_ELEM(*point, double, 0, 1) + h_matrix[4] * CV_MAT_ELEM(*point, double, 1, 1) + h_matrix[5] * CV_MAT_ELEM(*point, double, 2, 1);
	pz = h_matrix[6] * CV_MAT_ELEM(*point, double, 0, 1) + h_matrix[7] * CV_MAT_ELEM(*point, double, 1, 1) + h_matrix[8] * CV_MAT_ELEM(*point, double, 2, 1);
	npx = px / pz;
	npy = py / pz;
	CV_MAT_ELEM(*t_point, double, 0, 2) = p_transform[0] * npx + p_transform[2] + c_x - 1;
	CV_MAT_ELEM(*t_point, double, 0, 3) = p_transform[4] * npy + p_transform[5] + c_y - 1;

	px = h_matrix[0] * CV_MAT_ELEM(*point, double, 0, 2) + h_matrix[1] * CV_MAT_ELEM(*point, double, 1, 2) + h_matrix[2] * CV_MAT_ELEM(*point, double, 2, 2);
	py = h_matrix[3] * CV_MAT_ELEM(*point, double, 0, 2) + h_matrix[4] * CV_MAT_ELEM(*point, double, 1, 2) + h_matrix[5] * CV_MAT_ELEM(*point, double, 2, 2);
	pz = h_matrix[6] * CV_MAT_ELEM(*point, double, 0, 2) + h_matrix[7] * CV_MAT_ELEM(*point, double, 1, 2) + h_matrix[8] * CV_MAT_ELEM(*point, double, 2, 2);
	npx = px / pz;
	npy = py / pz;
	CV_MAT_ELEM(*t_point, double, 0, 4) = p_transform[0] * npx + p_transform[2] + c_x - 1;
	CV_MAT_ELEM(*t_point, double, 0, 5) = p_transform[4] * npy + p_transform[5] + c_y - 1;

	px = h_matrix[0] * CV_MAT_ELEM(*point, double, 0, 3) + h_matrix[1] * CV_MAT_ELEM(*point, double, 1, 3) + h_matrix[2] * CV_MAT_ELEM(*point, double, 2, 3);
	py = h_matrix[3] * CV_MAT_ELEM(*point, double, 0, 3) + h_matrix[4] * CV_MAT_ELEM(*point, double, 1, 3) + h_matrix[5] * CV_MAT_ELEM(*point, double, 2, 3);
	pz = h_matrix[6] * CV_MAT_ELEM(*point, double, 0, 3) + h_matrix[7] * CV_MAT_ELEM(*point, double, 1, 3) + h_matrix[8] * CV_MAT_ELEM(*point, double, 2, 3);
	npx = px / pz;
	npy = py / pz;
	CV_MAT_ELEM(*t_point, double, 0, 6) = p_transform[0] * npx + p_transform[2] + c_x - 1;
	CV_MAT_ELEM(*t_point, double, 0, 7) = p_transform[4] * npy + p_transform[5] + c_y - 1;
}

double PFSL3::elem_sum(double* input, int N) {
	double sum = 0;
	for(int i = 0; i < N; i++) {
		sum = sum + input[i];
	}

	return sum;
}

void PFSL3::elem_div(double* input, double div, double *output, int N) {
	for(int i = 0; i < N; i++) {
		output[i] = input[i] / div;
	}
}

// Sample Mean
void PFSL3::sample_mean(double* X_par, double* X_max, int N, double* Mean_X) {
	double diff_SL3[8];
	double delta = 100000000;
	double SL3_sample[9];
	double inv_SL3_mean[9];
	double SL3_mean[9];
	CvMat* temp_vec = cvCreateMat(1, 8, CV_64F);
	double SL3_temp[9];
	double SL3_temp_vec[8];
	double SL3_mat[9];
	double SL3_exp[9];
	double log_SL3[9];
	double v;

	int iter = 0;
	memcpy(&SL3_mean[0], X_max, 9 * 8);
	while(delta > 0.0001) {
		diff_SL3[0] = 0;
		diff_SL3[1] = 0;
		diff_SL3[2] = 0;
		diff_SL3[3] = 0;
		diff_SL3[4] = 0;
		diff_SL3[5] = 0;
		diff_SL3[6] = 0;
		diff_SL3[7] = 0;

		mat_inv_3x3(&SL3_mean[0], &inv_SL3_mean[0]);
		for(int i = 0; i < N; i++) {
			memcpy(&SL3_sample[0], &X_par[i * 9], 9 * 8);
			mat_mul_3x3(&inv_SL3_mean[0], &SL3_sample[0], &SL3_temp[0]);
			mat_log(&SL3_temp[0], &log_SL3[0]);
			sl3_to_vec(&log_SL3[0], &SL3_temp_vec[0]);

			diff_SL3[0] = diff_SL3[0] + SL3_temp_vec[0];
			diff_SL3[1] = diff_SL3[1] + SL3_temp_vec[1];
			diff_SL3[2] = diff_SL3[2] + SL3_temp_vec[2];
			diff_SL3[3] = diff_SL3[3] + SL3_temp_vec[3];
			diff_SL3[4] = diff_SL3[4] + SL3_temp_vec[4];
			diff_SL3[5] = diff_SL3[5] + SL3_temp_vec[5];
			diff_SL3[6] = diff_SL3[6] + SL3_temp_vec[6];
			diff_SL3[7] = diff_SL3[7] + SL3_temp_vec[7];
		}
		diff_SL3[0] = diff_SL3[0] / N;
		diff_SL3[1] = diff_SL3[1] / N;
		diff_SL3[2] = diff_SL3[2] / N;
		diff_SL3[3] = diff_SL3[3] / N;
		diff_SL3[4] = diff_SL3[4] / N;
		diff_SL3[5] = diff_SL3[5] / N;
		diff_SL3[6] = diff_SL3[6] / N;
		diff_SL3[7] = diff_SL3[7] / N;

		memcpy(&SL3_temp_vec[0], &diff_SL3[0], 8 * 8);
		sl3(&SL3_temp_vec[0], &SL3_mat[0]);
		mat_exp(&SL3_mat[0], &SL3_exp[0]);
		memcpy(&SL3_temp[0], &SL3_mean[0], 9 * 8);
		mat_mul_3x3(&SL3_temp[0], &SL3_exp[0], &SL3_mean[0]);
		memcpy(&SL3_temp_vec[0], &diff_SL3[0], 8 * 8);

		memcpy((double*)(temp_vec->data.ptr), &SL3_temp_vec[0], 8 * 8);
		v = cvDotProduct(temp_vec, temp_vec);
		delta = sqrt(v);

		iter++;

	}
	memcpy(Mean_X, &SL3_mean[0], 9 * 8);
	cvReleaseMat(&temp_vec);
	//cout << iter <<"iterations, delta ="<< delta << endl;
}

void PFSL3::mat_inv_3x3(double* input, double* output) {
	double Det;

	Det = input[0] * (input[8] * input[4] - input[7] * input[5]) - input[3] * (input[8] * input[1] - input[7] * input[2]) + input[6] * (input[5] * input[1] - input[4] * input[2]);

	output[0] = (input[8] * input[4] - input[7] * input[5]) / Det;
	output[3] = -(input[8] * input[3] - input[6] * input[5]) / Det;
	output[6] = (input[7] * input[3] - input[6] * input[4]) / Det;
	output[1] = -(input[8] * input[1] - input[7] * input[2]) / Det;
	output[4] = (input[8] * input[0] - input[6] * input[2]) / Det;
	output[7] = -(input[7] * input[0] - input[6] * input[1]) / Det;
	output[2] = (input[5] * input[1] - input[4] * input[2]) / Det;
	output[5] = -(input[5] * input[0] - input[3] * input[2]) / Det;
	output[8] = (input[4] * input[0] - input[3] * input[1]) / Det;
}

void PFSL3::cal_mean_img(float* input, int length, int num_img, float* output) {
	float* mean_img;
	mean_img = new float[length];
	for(int i = 0; i < length; i++) {
		mean_img[i] = 0;
	}

	for(int i = 0; i < num_img; i++) {
		for(int j = 0; j < length; j++) {
			mean_img[j] = mean_img[j] + input[length * i + j] / ((float)(num_img));
		}
	}
	memcpy(&output[0], &mean_img[0], length * 4);

	delete[] mean_img;
	mean_img = NULL;
}

void PFSL3::cal_mean_adj_img(float* input, float* mean_img, int length, int num_img, float* output) {
	float* diff_img;
	diff_img = new float[length];

	for(int i = 0; i < num_img; i++) {
		for(int j = 0; j < length; j++) {
			diff_img[j] = input[length * i + j] - mean_img[j];
		}
		memcpy(&output[length * i], &diff_img[0], length * 4);
	}

	delete[] diff_img;
	diff_img = NULL;
}

int PFSL3::find_index(CvMat* W, double threshold) {
	CvSize svd_size = cvGetSize(W);

	float sum = 0;
	for(int i = 0; i < svd_size.height; i++) {
		sum = sum + CV_MAT_ELEM(*W, float, i, i);
	}

	float temp = 0;
	int index;
	for(int i = 0; i < svd_size.height; i++) {
		temp = temp + CV_MAT_ELEM(*W, float, i, i);
		if(temp / sum >= threshold) {
			index = i + 1;
			break;
		}
	}

	return index;
}

void PFSL3::mean_update(float* mean_img, float* mean_update_img, float ff, int update_period, int num_tracked_img, int length) {
	float* new_mean;
	new_mean = new float[length];

	for(int i = 0; i < length; i++) {
		new_mean[i] = (num_tracked_img - update_period) * ff / ((num_tracked_img - update_period) * ff + update_period) * mean_img[i] + update_period / ((num_tracked_img - update_period) * ff + update_period) * mean_update_img[i];
	}
	memcpy(&mean_img[0], &new_mean[0], length * 4);

	delete[] new_mean;
	new_mean = NULL;
}

void PFSL3::img_diff_scalar_mul(float* input_A, float* input_B, float scalar, int length, float* output) {
	for(int i = 0; i < length; i++) {
		output[i] = (input_A[i] - input_B[i]) * scalar;
	}
}

void PFSL3::jacobian_calculation(double* p_matrix, CvMat* grad_x, CvMat* grad_y, CvMat* dX_du, int p_x, int p_y, CvMat* jacobian) {
	CvMat* temp1 = cvCreateMat(2, 9, CV_32F);
	CvMat* temp2 = cvCreateMat(2, 8, CV_32F);
	CvMat* temp3 = cvCreateMat(1, 2, CV_32F);
	CvMat* temp4 = cvCreateMat(1, 8, CV_32F);

	for(int i = 0; i < p_x; i++) {
		for(int j = 0; j < p_y; j++) {
			CV_MAT_ELEM(*temp1, float, 0, 0) = (float)p_matrix[0 + (i * p_y + j) * 2];
			CV_MAT_ELEM(*temp1, float, 0, 1) = 0;
			CV_MAT_ELEM(*temp1, float, 0, 2) = ((float)(-1) * (p_matrix[0 + (i * p_y + j) * 2] * p_matrix[0 + (i * p_y + j) * 2]));
			CV_MAT_ELEM(*temp1, float, 0, 3) = (float)p_matrix[1 + (i * p_y + j) * 2];
			CV_MAT_ELEM(*temp1, float, 0, 4) = 0;
			CV_MAT_ELEM(*temp1, float, 0, 5) = ((float)(-1) * (p_matrix[0 + (i * p_y + j) * 2] * p_matrix[1 + (i * p_y + j) * 2]));
			CV_MAT_ELEM(*temp1, float, 0, 6) = 1;
			CV_MAT_ELEM(*temp1, float, 0, 7) = 0;
			CV_MAT_ELEM(*temp1, float, 0, 8) = ((float)(-1) * p_matrix[0 + (i * p_y + j) * 2]);

			CV_MAT_ELEM(*temp1, float, 1, 0) = 0;
			CV_MAT_ELEM(*temp1, float, 1, 1) = (float)p_matrix[0 + (i * p_y + j) * 2];
			CV_MAT_ELEM(*temp1, float, 1, 2) = ((float)(-1) * (p_matrix[0 + (i * p_y + j) * 2] * p_matrix[1 + (i * p_y + j) * 2]));
			CV_MAT_ELEM(*temp1, float, 1, 3) = 0;
			CV_MAT_ELEM(*temp1, float, 1, 4) = (float)p_matrix[1 + (i * p_y + j) * 2];
			CV_MAT_ELEM(*temp1, float, 1, 5) = ((float)(-1) * (p_matrix[1 + (i * p_y + j) * 2] * p_matrix[1 + (i * p_y + j) * 2]));
			CV_MAT_ELEM(*temp1, float, 1, 6) = 0;
			CV_MAT_ELEM(*temp1, float, 1, 7) = 1;
			CV_MAT_ELEM(*temp1, float, 1, 8) = ((float)(-1) * p_matrix[1 + (i * p_y + j) * 2]);

			cvMatMul(temp1, dX_du, temp2);

			CV_MAT_ELEM(*temp3, float, 0, 0) = CV_MAT_ELEM(*grad_x, float, i, j);
			CV_MAT_ELEM(*temp3, float, 0, 1) = CV_MAT_ELEM(*grad_y, float, i, j);

			cvMatMul(temp3, temp2, temp4);

			memcpy((float*)(jacobian->data.ptr + jacobian->step * (i * p_y + j)), (float*)(temp4->data.ptr), 8 * 4);
		}
	}
	cvReleaseMat(&temp1);
	cvReleaseMat(&temp2);
	cvReleaseMat(&temp3);
	cvReleaseMat(&temp4);
}

void PFSL3::image_gradient(CvMat* input, int p_x, int p_y, CvMat* grad_x, CvMat* grad_y) {
	for(int i = 0; i < p_x; i++) {
		CV_MAT_ELEM(*grad_y, float, i, 0) = CV_MAT_ELEM(*input, float, i, 1) - CV_MAT_ELEM(*input, float, i, 0);
		CV_MAT_ELEM(*grad_y, float, i, p_y - 1) = CV_MAT_ELEM(*input, float, i, p_y - 1) - CV_MAT_ELEM(*input, float, i, p_y - 2);
		for(int j = 1; j < p_y - 1; j++) {
			CV_MAT_ELEM(*grad_y, float, i, j) = (CV_MAT_ELEM(*input, float, i, j + 1) - CV_MAT_ELEM(*input, float, i, j - 1)) / 2;
		}
	}

	for(int i = 0; i < p_y; i++) {
		CV_MAT_ELEM(*grad_x, float, 0, i) = CV_MAT_ELEM(*input, float, 1, i) - CV_MAT_ELEM(*input, float, 0, i);
		CV_MAT_ELEM(*grad_x, float, p_x - 1, i) = CV_MAT_ELEM(*input, float, p_x - 1, i) - CV_MAT_ELEM(*input, float, p_x - 2, i);
		for(int j = 1; j < p_x - 1; j++) {
			CV_MAT_ELEM(*grad_x, float, j, i) = (CV_MAT_ELEM(*input, float, j + 1, i) - CV_MAT_ELEM(*input, float, j - 1, i)) / 2;
		}
	}
}

unsigned long PFSL3::GetRDTSC(void) {
	unsigned int tmp1 = 0;
	unsigned int tmp2 = 0;
	unsigned long ret = 0;

#ifdef _WIN32
	__asm {
		RDTSC;
		mov tmp1, eax;
		mov tmp2, edx;
	}
#else
	__asm__("RDTSC;");
	__asm__("mov %0, %%eax;"
		: "=r"(tmp1));
	__asm__("mov %0, %%edx;"
		: "=r"(tmp2));
#endif
	//printf("tmp1: %d\n", tmp1);
	//printf("tmp2: %d\n", tmp2);

	ret = tmp2;
	long ret_val = ((ret << 32) | tmp1);
	//printf("ret_val: %ld\n", ret_val);
	return ret_val;
}
