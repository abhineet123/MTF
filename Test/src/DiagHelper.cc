#include "mtf/Test/Diagnostics.h"

_MTF_BEGIN_NAMESPACE


void Diagnostics::initializePixJacobian(){
	switch(params.update_type){
	case UpdateType::Additive:
		am->initializePixGrad(ssm->getPts());
		break;
	case UpdateType::Compositional:
		//ssm->initializeGradPts(am->getGradOffset());
		//am->initializePixGrad(ssm->getGradPts());
		am->initializePixGrad(ssm->getPts());
		break;
	}
}

void Diagnostics::updateInitPixJacobian(){
	switch(params.update_type){
	case UpdateType::Additive:
		am->initializePixGrad(ssm->getPts());
		ssm->cmptPixJacobian(init_pix_jacobian, am->getInitPixGrad());
		break;
	case UpdateType::Compositional:
		//ssm->updateGradPts(am->getGradOffset());
		//am->initializePixGrad(ssm->getGradPts());
		//ssm->cmptInitPixJacobian(init_pix_jacobian, am->getInitPixGrad());
		am->initializePixGrad(ssm->getPts());
		ssm->cmptWarpedPixJacobian(init_pix_jacobian, am->getInitPixGrad());
		break;
	}
}


void Diagnostics::updateCurrPixJacobian(){
	switch(params.update_type){
	case UpdateType::Additive:
		am->updatePixGrad(ssm->getPts());
		ssm->cmptPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
		break;
	case UpdateType::Compositional:
		//ssm->updateGradPts(am->getGradOffset());
		//am->updatePixGrad(ssm->getGradPts());
		//ssm->cmptInitPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
		am->updatePixGrad(ssm->getPts());
		ssm->cmptWarpedPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
		break;
	}
}

void Diagnostics::initializePixHessian(){
	switch(params.update_type){
	case UpdateType::Additive:
		am->initializePixHess(ssm->getPts());
		break;
	case UpdateType::Compositional:
		ssm->initializeHessPts(am->getHessOffset());
		am->initializePixHess(ssm->getPts(), ssm->getHessPts());
		break;
	}
}

void Diagnostics::updateInitPixHessian(){
	switch(params.update_type){
	case UpdateType::Additive:
		am->initializePixHess(ssm->getPts());
		ssm->cmptPixHessian(init_pix_hessian, am->getInitPixHess(), am->getInitPixGrad());
		break;
	case UpdateType::Compositional:
		//ssm->updateHessPts(am->getHessOffset());
		//am->initializePixHess(ssm->getPts(), ssm->getHessPts());
		//ssm->cmptInitPixHessian(init_pix_hessian, am->getInitPixHess(), am->getInitPixGrad());

		am->initializePixHess(ssm->getPts());
		ssm->cmptWarpedPixHessian(init_pix_hessian, am->getInitPixHess(), am->getInitPixGrad());
		break;
	}
}

void Diagnostics::updateCurrPixHessian(){
	switch(params.update_type){
	case UpdateType::Additive:
		am->updatePixHess(ssm->getPts());
		ssm->cmptPixHessian(curr_pix_hessian, am->getCurrPixHess(), am->getCurrPixGrad());
		break;
	case UpdateType::Compositional:
		//ssm->updateHessPts(am->getHessOffset());
		//am->updatePixHess(ssm->getPts(), ssm->getHessPts());
		//ssm->cmptInitPixHessian(curr_pix_hessian, am->getCurrPixHess(), am->getCurrPixGrad());

		am->updatePixHess(ssm->getPts());
		ssm->cmptWarpedPixHessian(curr_pix_hessian, am->getCurrPixHess(), am->getCurrPixGrad());
		break;
	}
}


void Diagnostics::updateInitSimilarity(){	
	am->initializePixVals(ssm->getPts());
	am->initializeSimilarity();
	am->updateSimilarity(false);
}


void Diagnostics::updateInitGrad(){
	am->initializePixVals(ssm->getPts());
	am->initializeSimilarity();
	am->initializeGrad();
	am->updateSimilarity();
	am->updateInitGrad();
}


void Diagnostics::updateInitHess(){
	am->initializePixVals(ssm->getPts());
	am->initializeSimilarity();
	am->initializeGrad();
	am->initializeHess();
	am->updateSimilarity();
	am->updateInitGrad();
}


void Diagnostics::updateCurrSimilarity(){
	am->updatePixVals(ssm->getPts());
	am->updateSimilarity(false);
}


void Diagnostics::updateCurrGrad(){
	am->updatePixVals(ssm->getPts());
	am->updateSimilarity();
	am->updateCurrGrad();
}


void Diagnostics::updateInitSelfHess(){
	// don't need curr_pix_vals to compute init self hessian
	// so can safely overwrite with the values extracted from init_img
	am->updatePixVals(ssm->getPts());
	am->updateSimilarity();
}

void Diagnostics::updateCurrSelfHess(){
	am->updatePixVals(ssm->getPts());
	am->updateSimilarity();
}


void Diagnostics::updateSSM(const VectorXd &state_update){
	switch(params.update_type){
	case UpdateType::Additive:
		ssm->additiveUpdate(state_update);
		break;
	case UpdateType::Compositional:
		ssm->compositionalUpdate(state_update);
		break;
	}
}

void Diagnostics::resetSSM(const VectorXd &state_update){
	switch(params.update_type){
	case UpdateType::Additive:
		ssm->additiveUpdate(-state_update);
		break;
	case UpdateType::Compositional:
		ssm->invertState(inv_ssm_state, state_update);
		ssm->compositionalUpdate(inv_ssm_state);
		break;
	}
}

void Diagnostics::updateAM(const VectorXd &state_update){
	am->updateState(state_update);
}

void Diagnostics::resetAM(const VectorXd &state_update){
	am->invertState(inv_am_state, state_update);
	am->updateState(inv_am_state);


}
void Diagnostics::updateState(const VectorXd &state_update, unsigned int state_id){
	if(state_id < ssm_state_size){
		VectorXd ssm_state_update = state_update.head(ssm_state_size);
		updateSSM(ssm_state_update);
	} else{
		VectorXd am_state_update = state_update.tail(am_state_size);
		updateAM(am_state_update);
	}}

void Diagnostics::resetState(const VectorXd &state_update, unsigned int state_id){	if(state_id < ssm_state_size){
		VectorXd ssm_state_update = state_update.head(ssm_state_size);
		resetSSM(ssm_state_update);
	} else{
		VectorXd am_state_update = state_update.tail(am_state_size);
		resetAM(am_state_update);
	}
}


_MTF_END_NAMESPACE

