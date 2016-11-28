#include "mtf/SM/NT/AESM.h"

_MTF_BEGIN_NAMESPACE

namespace nt{

	AESM::AESM(AM _am, SSM _ssm, const ParamType *aesm_params) :
		ESM(_am, _ssm, aesm_params){
		printf("Using Additive variant\n");
		name = "aesm_nt";
		log_fname = "log/aesm_log.txt";
		time_fname = "log/aesm_times.txt";
	}

	void AESM::initializePixJacobian(){
		am->initializePixGrad(ssm->getPts());
		ssm->cmptPixJacobian(init_pix_jacobian, am->getInitPixGrad());
	}


	void AESM::updatePixJacobian(){
		// compute pixel gradient of the current image warped with the current warp
		am->updatePixGrad(ssm->getPts());
		record_event("am->updatePixGrad");

		// multiply the pixel gradient with the SSM Jacobian to get the Jacobian of pixel values w.r.t. SSM parameters
		ssm->cmptPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
		record_event("ssm->cmptPixJacobian");

		ssm->cmptPixJacobian(init_pix_jacobian, am->getInitPixGrad());
		record_event("ssm->cmptPixJacobian (init)");
	}


	void AESM::initializePixHessian(){
		am->initializePixHess(ssm->getPts());
		ssm->cmptPixHessian(init_pix_hessian, am->getInitPixHess(), am->getInitPixGrad());
	}


	void AESM::updatePixHessian(){
		am->updatePixHess(ssm->getPts());
		record_event("am->updatePixHess");

		ssm->cmptPixHessian(curr_pix_hessian, am->getCurrPixHess(), am->getCurrPixGrad());
		record_event("ssm->cmptPixHessian");

		ssm->cmptPixHessian(init_pix_hessian, am->getInitPixHess(), am->getInitPixGrad());
		record_event("ssm->cmptPixHessian (init)");
	}


	void AESM::updateState(){
		ssm->additiveUpdate(ssm_update);
		record_event("ssm->additiveUpdate");
	}
}

_MTF_END_NAMESPACE
