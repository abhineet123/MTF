#ifndef MTF_PFSL3_H
#define MTF_PFSL3_H

#include <ctime>
#include <opencv/cv.h>
#include "mtf/TrackerBase.h"

//#include <inttypes.h>
//typedef int64_t __int64;

struct PFSL3Params {
	int p_x, p_y;
	std::vector<double>state_std;
	double rot, NCC_std, PCA_std, AR_p;
	int N, N_c, N_iter, sampling, capture, mean_check, outlier_flag;
	int len, init_size, update_period;
	float ff;
	double basis_thr;
	int max_num_basis, max_num_used_basis;
	bool show_weights, show_templates, debug_mode;
	PFSL3Params(int _p_x, int _p_y,
		const std::vector<double> &_state_sig,
		double _rot, double _NCC_std, double _PCA_std, double _AR_p,
		int _N, int _N_c, int _N_iter, int _sampling, int _capture,
		int _mean_check, int _outlier_flag, int _len,
		int _init_size, int _update_period, float _ff,
		double _basis_thr, int _max_num_basis, int _max_num_used_basis,
		bool show_weights, bool show_templates, bool _debug_mode);
	PFSL3Params(const PFSL3Params *params = nullptr);
};

class PFSL3 : public mtf::TrackerBase {
public:
	typedef PFSL3Params ParamType;

	PFSL3(const ParamType *pfsl3_params=nullptr);
	virtual ~PFSL3() {}
	void setImage(const cv::Mat &_img) override;
	int inputType() const  override{ return CV_32FC1; }
	void initialize(const cv::Mat&cv_corners) override;
	void update() override;
	void updateCVCorners();

private:
	ParamType params;

	CvMat curr_img;
	CvMat *img;

	FILE *debug_fid;
	int p_x, p_y;
	double c_x, c_y, l_x, l_y;
	int frame_id;

	CvSize frame_size;
	CvMat *c_p, *rc_p, *temp_cp, *temp_rcp, *inv_temp_rcp, *transform;
	double p_transform[6], *p_matrix, init_h[9];
	CvMat* obj_template, warped_img_2D_header, *warped_img_2D;

	CvPoint p1, p2, p3, p4;
	CvMat *dX_du, *grad_x, *grad_y, *jacobian, *adj_template;
	CvScalar mean_v;

	double *X_par, *X_pred, *AR_velocity, *AR_pred, *mean_X_par, *prob, *w;
	double u, v, NCC, dist, prob_sum, msec;

	double random_coeff[8], scaled_rand[8], sl3_rand[9], previous_X[9];
	double pred_X[9], AR[9], log_AR[9], a_log_AR[9], _update[9];
	double update_exp[9], pred_AR[9], inv_X[9];

	double *Mean, *Cov, state_cov[64];
	double diff1, X_diff[8], diff2, *pred_measure;
	double NCC_real, PCA_real;

	CvMat* Init_Cov, *NCC_jacobian, *NCC_J_temp1, *NCC_J_temp2;
	CvMat* sigma_11, *sigma_12, *sigma_22, *temp_like_NCC;
	double mu_2;

	int best_index;
	double New_Cov[64], New_Mean[9], Prev_Cov[64], Prev_Mean[9];
	int num_tracked_img, num_basis, num_used_basis;

	CvMat* basis, *projected, *data_proj, *diff_img, *temp_basis;

	float proj_sum;
	float* tracked_img;
	float *mean_img, *mean_update_img, *mean_adj_img, *update_img, *new_data;

	int num_update_img;	
	double PCA;
	int PCA_flag;

	double max_X[9], increment[8], update_NCC_thr;

	CvMat* cv_mean_img, *data_res, *templ, *divide;
	CvRNG rng_state;
	CvMat* random, *previous_X3x3, *warped_img, *adj_warped_img, *divide_prob;
	CvMat* temp_MAT, *temp_w;
	CvMat* inv_sigma_22, *temp_Cov1, *temp_Cov2, *temp_increment, *temp_pred_measure;
	CvPoint min_loc, max_loc;

	char UPLO;

	double X_par_temp[9], dist1, dist2, dist3, min_v, max_v, t_mu_2[2];

	CvMat *CH, *colored_random, *dist3_temp1, *dist3_temp2, *COV3, *inv_COV3;
	CvPoint min_ind, max_ind;
	CvMat *X_diff_cv, *PCA_recon, *PCA_recon_weight, *PCA_recon_temp, *mean_jacobian, *d_PCA_measure;
	CvMat *PCA_jacobian, *total_jacobian, *t_sigma_12, *t_sigma_22, *total_like, *inv_t_sigma_22, *t_temp_Cov1;
	CvMat *recon_error, *dist2_temp1, *dist2_temp2, *COV2, *inv_COV2;

	double update_PCA_thr, num_est, outlier_thr, update_thr;
	float weight;
	int num_outlier;

	CvMat *QR_Q, *Q_proj, *W, *U, *V, *A, *Sing_val;
	CvMat	*bestindex_plot, *weight_plot;
	int* outindex, *outindex2;

	float* temp_img;

	double *X_par_mean, *AR_velocity_mean, *new_NCC_sig;
	double NCC_J_norm, PCA_J_norm;

	void imwarping_NN(CvMat* img, double* h_matrix, double* p_matrix, double* p_transform, int c_x, int c_y, int p_x, int p_y, CvSize frame_size, CvMat* warped_img);
	void imwarping_BL(CvMat* img, double* h_matrix, double* p_matrix, double* p_transform, int c_x, int c_y, int p_x, int p_y, CvSize frame_size, CvMat* warped_img);
	void point_transform(const CvMat* point, const double* h_matrix, const double* p_transform,
		int c_x, int c_y, CvMat* t_point);
	void mat_exp(double* input, double* output);
	void mat_log(double* input, double* output);
	void mat_mul_3x3(double* input_A, double* input_B, double* output);
	void sl3_to_vec(double* input, double* output);
	void sl3(double* input, double* output);
	void mat_scalar(double* input, double scalar, double* output);
	void mat_add(double* input_A, double* input_B, double* output);
	void mat_mul_elem(double* input_A, double* input_B, double* output);
	void mat_display(double* input, int size);
	void resampling(double* w, int N, int M, int* outindex);
	void particle_copy(int* outindex, int N, double* input_A, double* input_B, double* output_A, double* output_B);
	double elem_sum(double* input, int N);
	void elem_div(double* input, double div, double *output, int N);
	void sample_mean(double* X_par, double* X_max, int N, double* Mean_X);
	void mat_inv_3x3(double* input, double* output);
	void cal_mean_img(float* input, int length, int num_img, float* output);
	void cal_mean_adj_img(float* input, float* mean_img, int length, int num_img, float* output);
	int find_index(CvMat* W, double threshold);
	void mean_update(float* mean_img, float* mean_update_img, float ff, int update_period, int num_tracked_img, int length);
	void img_diff_scalar_mul(float* input_A, float* input_B, float scalar, int length, float* output);
	void jacobian_calculation(double* p_matrix, CvMat* grad_x, CvMat* grad_y, CvMat* dX_du, int p_x, int p_y, CvMat* jacobian);
	void image_gradient(CvMat* input, int p_x, int p_y, CvMat* grad_x, CvMat* grad_y);
	unsigned long GetRDTSC(void);
};

#endif

