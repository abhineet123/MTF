#ifndef MTF_DIAG_BASE_H
#define MTF_DIAG_BASE_H

#include "mtf/Macros/common.h"

#define RES_X 50
#define RES_Y 50

_MTF_BEGIN_NAMESPACE

class DiagBase{
public:

	enum class AnalyticalDataType {
		Norm, FeatNorm,
		// Jacobians
		StdJac, ESMJac, DiffOfJacs,
		// Hessians
		Std, ESM, InitSelf, CurrSelf,
		Std2, ESM2, InitSelf2, CurrSelf2,
		SumOfStd, SumOfStd2, SumOfSelf, SumOfSelf2
	};
	enum class NumericalDataType {
		Jacobian, Hessian, NHessian
	};

	typedef AnalyticalDataType ADT;
	typedef NumericalDataType NDT;

	int am_dist_size;
	int ssm_state_size;

	MatrixXd diagnostics_data;

	DiagBase() : am_dist_size(0), ssm_state_size(0){}
	DiagBase(const cv::Mat &init_img) : am_dist_size(0), ssm_state_size(0){}
	virtual ~DiagBase(){}

	virtual void setImage(const cv::Mat &img) = 0;
	virtual void initialize(const cv::Mat &corners) = 0;
	virtual void update(const cv::Mat &corners) = 0;
	virtual void initialize(const cv::Mat &img, const cv::Mat &corners){
		setImage(img);
		initialize(corners);
	}
	virtual void update(const cv::Mat &img, const cv::Mat &corners){
		setImage(img);
		update(corners);
	}

	virtual void generateAnalyticalData(VectorXd &param_range,
		int n_pts, AnalyticalDataType data_type, const char* fname) = 0;
	void generateAnalyticalData(double param_range,
		int n_pts, AnalyticalDataType data_type,
		const char* fname = nullptr){
		VectorXd param_range_vec(ssm_state_size);
		param_range_vec.fill(param_range);
		generateAnalyticalData(param_range_vec, n_pts,
			data_type, fname);
	}

	virtual void generateInverseAnalyticalData(VectorXd &param_range,
		int n_pts, AnalyticalDataType data_type, const char* fname = nullptr) = 0;
	void generateInverseAnalyticalData(double param_range,
		int n_pts, AnalyticalDataType data_type,
		const char* fname = nullptr){
		VectorXd param_range_vec(ssm_state_size);
		param_range_vec.fill(param_range);
		generateInverseAnalyticalData(param_range_vec, n_pts,
			data_type, fname);
	}

	virtual void generateNumericalData(VectorXd &param_range_vec, int n_pts,
		NumericalDataType data_type, const  char* fname, double grad_diff) = 0;
	virtual void generateInverseNumericalData(VectorXd &param_range_vec,
		int n_pts, NumericalDataType data_type, const char* fname, double grad_diff) = 0;

	virtual void generateSSMParamData(VectorXd &param_range_vec,
		int n_pts, const char* fname) = 0;

	const char* toString(AnalyticalDataType data_type);
	const char* toString(NumericalDataType data_type);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
_MTF_END_NAMESPACE

#endif
