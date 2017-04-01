#ifndef MTF_ESMH_H
#define MTF_ESMH_H

#include "SearchMethod.h"

#define ESMH_MAX_ITERS 10
#define ESMH_EPSILON 0.01
#define ESMH_JAC_TYPE 0
#define ESMH_HESS_TYPE 0
#define ESMH_SEC_ORD_HESS false
#define ESMH_ENABLE_SPI false
#define ESMH_SPI_THRESH 10
#define ESMH_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

struct ESMHParams{

	enum class JacType{ Original, DiffOfJacs };
	enum class HessType {
		Original, SumOfStd, SumOfSelf,
		InitialSelf, CurrentSelf, Std
	};

	int max_iters; //! maximum iterations of the ESMH algorithm to run for each frame
	double epsilon; //! maximum L1 norm of the state update vector at which to stop the iterations

	JacType jac_type;
	HessType hess_type;
	bool sec_ord_hess;

	bool enable_spi;
	double spi_thresh;
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	// value constructor
	ESMHParams(int _max_iters, double _epsilon, 
		JacType _jac_type, HessType _hess_type, bool _sec_ord_hess,
		bool _enable_spi, double _spi_thresh,
		bool _debug_mode){
		max_iters = _max_iters;
		epsilon = _epsilon;
		jac_type = _jac_type;
		hess_type = _hess_type;
		sec_ord_hess = _sec_ord_hess;
		enable_spi = _enable_spi;
		spi_thresh = _spi_thresh;
		debug_mode = _debug_mode;
	}
	// default and copy constructor
	ESMHParams(ESMHParams *params = nullptr) :
		max_iters(ESMH_MAX_ITERS), epsilon(ESMH_EPSILON),
		jac_type(static_cast<JacType>(ESMH_JAC_TYPE)),
		hess_type(static_cast<HessType>(ESMH_HESS_TYPE)),
		sec_ord_hess(ESMH_SEC_ORD_HESS),
		enable_spi(ESMH_ENABLE_SPI), spi_thresh(ESMH_SPI_THRESH),
		debug_mode(ESMH_DEBUG_MODE){
		if(params){
			max_iters = params->max_iters;
			epsilon = params->epsilon;
			jac_type = params->jac_type;
			hess_type = params->hess_type;
			sec_ord_hess = params->sec_ord_hess;
			enable_spi = params->enable_spi;
			spi_thresh = params->spi_thresh;
			debug_mode = params->debug_mode;
		}
	}
};

template<class AM, class SSM>
class ESMH : public SearchMethod < AM, SSM > {

public:
	typedef ESMHParams ParamType;
	ParamType params;

	typedef ParamType::JacType JacType;
	typedef ParamType::HessType HessType;

	using SearchMethod<AM, SSM> ::am;
	using SearchMethod<AM, SSM> ::ssm;
	using typename SearchMethod<AM, SSM> ::AMParams;
	using typename SearchMethod<AM, SSM> ::SSMParams;
	using SearchMethod<AM, SSM> ::cv_corners_mat;
	using SearchMethod<AM, SSM> ::name;
	using SearchMethod<AM, SSM> ::initialize;
	using SearchMethod<AM, SSM> ::update;

	int frame_id;
	VectorXc pix_mask2;
	VectorXb pix_mask;
	VectorXd rel_pix_diff;
	double max_pix_diff;

	Matrix24d prev_corners;
	//! N x S jacobians of the pixel values w.r.t the SSM state vector where N = resx * resy
	//! is the no. of pixels in the object patch
	MatrixXd init_pix_jacobian, curr_pix_jacobian, mean_pix_jacobian;
	MatrixXd init_pix_hessian, curr_pix_hessian, mean_pix_hessian;
	VectorXd ssm_update;
	//! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector
	RowVectorXd jacobian;
	//! S x S Hessian of the AM error norm w.r.t. SSM state vector
	MatrixXd hessian, init_self_hessian;


	ESMH(const ParamType *esm_params,
		const AMParams *am_params, const SSMParams *ssm_params) :
		SearchMethod<AM, SSM>(am_params, ssm_params),
		params(esm_params){
		printf("\n");
		printf("Using ESM tracker with:\n");
		printf("max_iters: %d\n", params.max_iters);
		printf("epsilon: %f\n", params.epsilon);
		printf("jac_type: %d\n", params.jac_type);
		printf("hess_type: %d\n", params.hess_type);
		printf("sec_ord_hess: %d\n", params.sec_ord_hess);
		printf("enable_spi: %d\n", params.enable_spi);
		printf("spi_thresh: %f\n", params.spi_thresh);
		printf("debug_mode: %d\n", params.debug_mode);

		printf("appearance model: %s\n", am.name.c_str());
		printf("state space model: %s\n", ssm.name.c_str());
		printf("\n");

		name = "esm";

		frame_id = 0;
		max_pix_diff = 0;

		switch(params.jac_type){
		case JacType::Original:
			printf("Using original ESM Jacobian\n");
			break;
		case JacType::DiffOfJacs:
			printf("Using Difference of Jacobians\n");
			break;
		default:
			throw std::invalid_argument("Invalid Jacobian type provided");
		}

		const char *hess_order = params.sec_ord_hess ? "Second" : "First";
		switch(params.hess_type){
		case HessType::Original:
			printf("Using %s order original ESM Hessian\n", hess_order);
			break;
		case HessType::SumOfStd:
			printf("Using Sum of %s order Standard Hessians\n", hess_order);
			break;
		case HessType::SumOfSelf:
			printf("Using Sum of %s order Self Hessians\n", hess_order);
			break;
		case HessType::InitialSelf:
			printf("Using %s order Initial Self Hessian\n", hess_order);
			break;
		case HessType::CurrentSelf:
			printf("Using %s order Current Self Hessian\n", hess_order);
			break;
		case HessType::Std:
			printf("Using %s order Standard Hessian\n", hess_order);
			break;
		default:
			throw std::invalid_argument("Invalid Hessian type provided");
		}

		ssm_update.resize(ssm.getStateSize());
		jacobian.resize(ssm.getStateSize());
		hessian.resize(ssm.getStateSize(), ssm.getStateSize());

		init_pix_jacobian.resize(am.getNPix(), ssm.getStateSize());
		curr_pix_jacobian.resize(am.getNPix(), ssm.getStateSize());

		if(params.jac_type == JacType::Original || params.hess_type == HessType::Original){
			mean_pix_jacobian.resize(am.getNPix(), ssm.getStateSize());
		}
		if(params.hess_type == HessType::SumOfSelf){
			init_self_hessian.resize(ssm.getStateSize(), ssm.getStateSize());
		}
		if(params.sec_ord_hess){
			init_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getNPix());
			if(params.hess_type != HessType::InitialSelf){
				curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getNPix());
				if(params.hess_type == HessType::Original){
					mean_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getNPix());
				}
			}
		}
	}

	void initialize(const cv::Mat &corners) override{
		frame_id = 0;
		ssm.initialize(corners);
		am.initializePixVals(ssm.getPts());

		if(params.enable_spi){
			if(!ssm.supportsSPI())
				throw std::domain_error("ESM::initialize : SSM does not support SPI");
			if(!am.supportsSPI())
				throw std::domain_error("ESM::initialize : AM does not support SPI");

			printf("Using Selective Pixel Integration\n");
			pix_mask.resize(am.getNPix());
			ssm.setSPIMask(pix_mask.data());
			am.setSPIMask(pix_mask.data());
		}

		ssm.initializeGradPts(am.getGradOffset());
		am.initializePixGrad(ssm.getGradPts());
		ssm.cmptInitPixJacobian(init_pix_jacobian, am.getInitPixGrad());

		if(params.sec_ord_hess){
			ssm.initializeHessPts(am.getHessOffset());
			am.initializePixHess(ssm.getPts(), ssm.getHessPts());
			ssm.cmptInitPixHessian(init_pix_hessian, am.getInitPixHess(), am.getInitPixGrad());
		}
		am.initializeSimilarity();
		am.initializeGrad();
		am.initializeHess();

		if(params.hess_type == HessType::InitialSelf || params.hess_type == HessType::SumOfSelf){
			if(params.sec_ord_hess){
				am.cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
			} else{
				am.cmptSelfHessian(hessian, init_pix_jacobian);
			}
			if(params.hess_type == HessType::SumOfSelf){
				init_self_hessian = hessian;
			}
		}
		ssm.getCorners(cv_corners_mat);
	}

	void update() override{
		++frame_id;
		am.setFirstIter();
		for(int iter_id = 0; iter_id < params.max_iters; iter_id++){
			// extract pixel values from the current image at the latest known position of the object
			am.updatePixVals(ssm.getPts());

			if(params.enable_spi){
				rel_pix_diff = (am.getInitPixVals() - am.getCurrPixVals()) / max_pix_diff;
				pix_mask = rel_pix_diff.cwiseAbs().array() < params.spi_thresh;
			}
			// compute pixel gradient of the current image warped with the current warp
			ssm.updateGradPts(am.getGradOffset());
			am.updatePixGrad(ssm.getGradPts());
			// multiply the pixel gradient with the SSM Jacobian to get the Jacobian of pixel values w.r.t. SSM parameters
			ssm.cmptInitPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());

			if(params.jac_type == JacType::Original || params.hess_type == HessType::Original){
				mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
			}
			if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
				ssm.updateHessPts(am.getHessOffset());
				am.updatePixHess(ssm.getPts(), ssm.getHessPts());
				ssm.cmptInitPixHessian(curr_pix_hessian, am.getCurrPixHess(), am.getCurrPixGrad());
			}
			// compute the prerequisites for the gradient functions
			am.updateSimilarity();
			// update the gradient of the error norm w.r.t. current pixel values
			am.updateCurrGrad();
			// update the gradient of the error norm w.r.t. initial pixel values
			am.updateInitGrad();

			switch(params.jac_type){
			case JacType::Original:
				// take the mean at the level of the jacobian of the pixel values wrt SSM parameters, 
				//then use this mean jacobian to compute the Jacobian of the error norm wrt SSM parameters
				am.cmptCurrJacobian(jacobian, mean_pix_jacobian);
				break;
			case JacType::DiffOfJacs:
				// compute the mean difference between the Jacobians of the error norm w.r.t. initial AND current values of SSM parameters
				am.cmptDifferenceOfJacobians(jacobian, init_pix_jacobian, curr_pix_jacobian);
				jacobian *= 0.5;
				break;
			}
			switch(params.hess_type){
			case HessType::InitialSelf:
				break;
			case HessType::Original:
				if(params.sec_ord_hess){
					mean_pix_hessian = (init_pix_hessian + curr_pix_hessian) / 2.0;
					am.cmptCurrHessian(hessian, mean_pix_jacobian, mean_pix_hessian);
				} else{
					am.cmptCurrHessian(hessian, mean_pix_jacobian);
				}
				break;
			case HessType::SumOfStd:
				if(params.sec_ord_hess){
					am.cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian,
						init_pix_hessian, curr_pix_hessian);
				} else{
					am.cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian);
				}
				hessian *= 0.5;
				break;
			case HessType::SumOfSelf:
				if(params.sec_ord_hess){
					am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
				} else{
					am.cmptSelfHessian(hessian, curr_pix_jacobian);
				}
				hessian = (hessian + init_self_hessian) * 0.5;
				break;
			case HessType::CurrentSelf:
				if(params.sec_ord_hess){
					am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
				} else{
					am.cmptSelfHessian(hessian, curr_pix_jacobian);
				}
				break;
			case HessType::Std:
				if(params.sec_ord_hess){
					am.cmptCurrHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
				} else{
					am.cmptCurrHessian(hessian, curr_pix_jacobian);
				}
				break;
			}
			ssm_update = -hessian.colPivHouseholderQr().solve(jacobian.transpose());
			prev_corners = ssm.getCorners();
			ssm.compositionalUpdate(ssm_update);
			double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
			if(update_norm < params.epsilon){
				break;
			}
			am.clearFirstIter();
		}
		ssm.getCorners(cv_corners_mat);
	}
};
_MTF_END_NAMESPACE

#endif

