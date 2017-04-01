#include "mtf/SM/NT/FCSD.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

FCSDParams::FCSDParams(int _max_iters, double _epsilon,
double _learning_rate, bool _debug_mode,
int _hess_type){
	max_iters = _max_iters;
	epsilon = _epsilon;
	learning_rate = _learning_rate;
	debug_mode = _debug_mode;
	hess_type = _hess_type;
}
FCSDParams::FCSDParams(const FCSDParams *params) :
max_iters(FCSD_MAX_ITERS),
epsilon(FCSD_EPSILON),
learning_rate(FCSD_REC_INIT_ERR_GRAD),
debug_mode(FCSD_DEBUG_MODE),
hess_type(FCSD_HESS_TYPE){
	if(params){
		max_iters = params->max_iters;
		epsilon = params->epsilon;
		learning_rate = params->learning_rate;
		debug_mode = params->debug_mode;
		hess_type = params->hess_type;
	}
}

namespace nt{

	FCSD::FCSD(AM _am, SSM _ssm,
		const ParamType *fcsd_params) :
		SearchMethod(_am, _ssm),
		params(fcsd_params) {

		printf("\n");
		printf("Using Forward Compositional Steepest Descent (NT) SM with:\n");
		printf("max_iters: %d\n", params.max_iters);
		printf("epsilon: %f\n", params.epsilon);
		printf("learning_rate: %f\n", params.learning_rate);
		printf("debug_mode: %d\n", params.debug_mode);
		printf("hess_type: %d\n", params.hess_type);
		printf("appearance model: %s\n", am->name.c_str());
		printf("state space model: %s\n", ssm->name.c_str());
		printf("\n");

		name = "fcsd_nt";
		log_fname = "log/mtf_fcsd_log.txt";
		time_fname = "log/mtf_fcsd_times.txt";
		frame_id = 0;

		printf("ssm->getStateSize(): %d\n", ssm->getStateSize());

		ssm_update.resize(ssm->getStateSize());
		curr_pix_jacobian.resize(am->getPatchSize(), ssm->getStateSize());
		hessian.resize(ssm->getStateSize(), ssm->getStateSize());
		jacobian.resize(ssm->getStateSize());
		learning_rate = params.learning_rate;
		printf("learning_rate: %f\n", learning_rate);

	}


	void FCSD::initialize(const cv::Mat &corners){
		start_timer();

		ssm->initialize(corners, am->getNChannels());
		am->initializePixVals(ssm->getPts());

		//am->initializePixGrad(ssm->getPts());

		ssm->initializeGradPts(am->getGradOffset());
		am->initializePixGrad(ssm->getGradPts());

		am->initializeSimilarity();
		am->initializeGrad();
		am->initializeHess();
		ssm->getCorners(cv_corners_mat);

		end_timer();
		write_interval(time_fname, "w");
	}


	void FCSD::update(){
		++frame_id;
		write_frame_id(frame_id);

		utils::printScalarToFile(frame_id, "\n\nframe_id", time_fname, "%6d", "a");
		double upd_time = 0;

		am->setFirstIter();
		for(int i = 0; i < params.max_iters; i++){
			init_timer();

			am->updatePixVals(ssm->getPts());
			record_event("am->updatePixVals");

			ssm->updateGradPts(am->getGradOffset());
			record_event("ssm->updateGradPts");

			am->updatePixGrad(ssm->getGradPts());
			record_event("am->updatePixGrad New");

			ssm->cmptPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
			record_event("ssm->cmptPixJacobian");

			am->updateSimilarity();
			record_event("am->updateSimilarity");

			am->updateCurrGrad();
			record_event("am->updateCurrGrad");

			am->cmptCurrJacobian(jacobian, curr_pix_jacobian);
			record_event("am->cmptCurrJacobian");

			am->cmptCurrHessian(hessian, curr_pix_jacobian);
			record_event("am->cmptCurrHessian");

			MatrixXd temp = (jacobian*jacobian.transpose()) / (jacobian*hessian*jacobian.transpose());
			learning_rate = temp(0, 0);

			if(params.debug_mode){
				utils::printMatrix(temp, "temp");
				utils::printScalar(learning_rate, "learning_rate");
			}

			ssm_update = -learning_rate*jacobian.transpose();
			record_event("ssm_update");

			prev_corners = ssm->getCorners();

			ssm->compositionalUpdate(ssm_update);
			record_event("ssm->compositionalUpdate");

			double update_norm = (prev_corners - ssm->getCorners()).squaredNorm();
			record_event("update_norm");


			if(update_norm < params.epsilon){
				if(params.debug_mode){
					printf("n_iters: %d\n", i + 1);
				}
				break;
			}
			am->clearFirstIter();
		}
		ssm->getCorners(cv_corners_mat);
	}
}

_MTF_END_NAMESPACE
