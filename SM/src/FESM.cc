#include "FESM.h"
_MTF_BEGIN_NAMESPACE

// ----------------------------------------------------------------------------------------//
// ---------------------------------- JacT::Original ---------------------------------- //
// ----------------------------------------------------------------------------------------//

template<class AM, class SSM>
FESM<AM, SSM, HessT::Original, JacT::Original> ::
FESM(const ParamType *esm_params, const AMParams *am_params, const SSMParams *ssm_params ) :
FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using original ESM Jacobian\n");
	printf("Using %s order original ESM Hessian\n", hess_order.c_str());
	mean_pix_jacobian.resize(am.getPixCount(), ssm.getStateSize());
	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}

template<class AM, class SSM>
FESM<AM, SSM, HessT::SumOfStd, JacT::Original> ::
FESM(const ParamType *esm_params,	const AMParams *am_params,const SSMParams *ssm_params ) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using original ESM Jacobian\n");
	printf("Using Sum of %s order Standard Hessians\n", hess_order.c_str());
	mean_pix_jacobian.resize(am.getPixCount(), ssm.getStateSize());
	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::SumOfStd, JacT::Original> ::updateHessian() {
	if(params.sec_ord_hess){
		am.cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian,
			init_pix_hessian, curr_pix_hessian);
		record_event("am.cmptSumOfHessians (second order)");
	} else{
		am.cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian);
		record_event("am.cmptSumOfHessians (first order)");
	}
	hessian *= 0.5;
}
template<class AM, class SSM>
FESM<AM, SSM, HessT::SumOfSelf, JacT::Original> ::
FESM(const ParamType *esm_params,const AMParams *am_params,const SSMParams *ssm_params ) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using original ESM Jacobian\n");
	printf("Using Sum of %s order Self Hessians\n", hess_order.c_str());
	mean_pix_jacobian.resize(am.getPixCount(), ssm.getStateSize());
	mean_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::SumOfSelf, JacT::Original> ::initializeHessian() {
	if(params.sec_ord_hess){
		am.cmptSelfHessian(init_self_hessian, init_pix_jacobian, init_pix_hessian);
	} else{
		am.cmptSelfHessian(init_self_hessian, init_pix_jacobian);
	}
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::SumOfSelf, JacT::Original> ::updateHessian() {
	if(params.sec_ord_hess){
		am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
		record_event("am.cmptSelfHessian (second order)");
	} else{
		am.cmptSelfHessian(hessian, curr_pix_jacobian);
		record_event("am.cmptSelfHessian (first order)");
	}
	hessian = (hessian + init_self_hessian) * 0.5;
}


template<class AM, class SSM>
FESM<AM, SSM, HessT::InitialSelf, JacT::Original> ::
FESM(const ParamType *esm_params,	const AMParams *am_params,const SSMParams *ssm_params) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using original ESM Jacobian\n");
	printf("Using %s order Initial Self Hessian\n", hess_order.c_str());
	mean_pix_jacobian.resize(am.getPixCount(), ssm.getStateSize());
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::InitialSelf, JacT::Original> ::initializeHessian(){
	if(params.sec_ord_hess){
		am.cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
	} else{
		am.cmptSelfHessian(hessian, init_pix_jacobian);
	}
}
template<class AM, class SSM>
FESM<AM, SSM, HessT::CurrentSelf, JacT::Original> ::
FESM(const ParamType *esm_params,	const AMParams *am_params,const SSMParams *ssm_params) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using original ESM Jacobian\n");
	printf("Using %s order Current Self Hessian\n", hess_order.c_str());
	mean_pix_jacobian.resize(am.getPixCount(), ssm.getStateSize());
	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::CurrentSelf, JacT::Original> ::updateHessian() {
	if(params.sec_ord_hess){
		am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
		record_event("am.cmptSelfHessian (second order)");
	} else{
		am.cmptSelfHessian(hessian, curr_pix_jacobian);
		record_event("am.cmptSelfHessian (first order)");
	}
}

template<class AM, class SSM>
FESM<AM, SSM, HessT::Std, JacT::Original> ::
FESM(const ParamType *esm_params,	const AMParams *am_params,const SSMParams *ssm_params) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using original ESM Jacobian\n");
	printf("Using %s order Standard Hessian\n", hess_order.c_str());
	mean_pix_jacobian.resize(am.getPixCount(), ssm.getStateSize());
	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::Std, JacT::Original> ::updateHessian() {
	if(params.sec_ord_hess){
		am.cmptCurrHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
		record_event("am.cmptCurrHessian (second order)");
	} else{
		am.cmptCurrHessian(hessian, curr_pix_jacobian);
		record_event("am.cmptCurrHessian (first order)");
	}
}


// ----------------------------------------------------------------------------------------//
// ---------------------------------- JacT::DiffOfJacs ---------------------------------- //
// ----------------------------------------------------------------------------------------//

#define update_jacobian(hess_type) \
	template<class AM, class SSM> \
	void FESM<AM, SSM, HessT::hess_type, JacT::DiffOfJacs> ::updateJacobian() { \
		am.cmptDifferenceOfJacobians(jacobian, init_pix_jacobian, curr_pix_jacobian); \
		jacobian *= 0.5; \
		record_event("am.cmptDifferenceOfJacobians"); \
	}
update_jacobian(Original)
update_jacobian(SumOfStd)
update_jacobian(SumOfSelf)
update_jacobian(InitialSelf)
update_jacobian(CurrentSelf)
update_jacobian(Std)


template<class AM, class SSM>
FESM<AM, SSM, HessT::Original, JacT::DiffOfJacs> ::
FESM(const ParamType *esm_params, const AMParams *am_params,
const SSMParams *ssm_params) :
FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using Difference of Jacobians\n");
	printf("Using %s order original ESM Hessian\n", hess_order.c_str());
	mean_pix_jacobian.resize(am.getPixCount(), ssm.getStateSize());
	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}
template<class AM, class SSM>
FESM<AM, SSM, HessT::SumOfStd, JacT::DiffOfJacs> ::
FESM(const ParamType *esm_params,	const AMParams *am_params,const SSMParams *ssm_params) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using Difference of Jacobians\n");
	printf("Using Sum of %s order Standard Hessians\n", hess_order.c_str());

	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::SumOfStd, JacT::DiffOfJacs> ::updateHessian() {
	if(params.sec_ord_hess){
		am.cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian,
			init_pix_hessian, curr_pix_hessian);
		record_event("am.cmptSumOfHessians (second order)");
	} else{
		am.cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian);
		record_event("am.cmptSumOfHessians (first order)");
	}
	hessian *= 0.5;
}

template<class AM, class SSM>
FESM<AM, SSM, HessT::SumOfSelf, JacT::DiffOfJacs> ::
FESM(const ParamType *esm_params,	const AMParams *am_params,const SSMParams *ssm_params) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using Difference of Jacobians\n");
	printf("Using Sum of %s order Self Hessians\n", hess_order.c_str());

	mean_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::SumOfSelf, JacT::DiffOfJacs> ::initializeHessian() {
	if(params.sec_ord_hess){
		am.cmptSelfHessian(init_self_hessian, init_pix_jacobian, init_pix_hessian);
	} else{
		am.cmptSelfHessian(init_self_hessian, init_pix_jacobian);
	}
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::SumOfSelf, JacT::DiffOfJacs> ::updateHessian() {
	if(params.sec_ord_hess){
		am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
		record_event("am.cmptSelfHessian (second order)");
	} else{
		am.cmptSelfHessian(hessian, curr_pix_jacobian);
		record_event("am.cmptSelfHessian (first order)");
	}
	hessian = (hessian + init_self_hessian) * 0.5;
}


template<class AM, class SSM>
FESM<AM, SSM, HessT::InitialSelf, JacT::DiffOfJacs> ::
FESM(const ParamType *esm_params,	const AMParams *am_params, const SSMParams *ssm_params) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using Difference of Jacobians\n");
	printf("Using %s order Initial Self Hessian\n", hess_order.c_str());

}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::InitialSelf, JacT::DiffOfJacs> ::initializeHessian(){
	if(params.sec_ord_hess){
		am.cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
	} else{
		am.cmptSelfHessian(hessian, init_pix_jacobian);
	}
}

template<class AM, class SSM>
FESM<AM, SSM, HessT::CurrentSelf, JacT::DiffOfJacs> ::
FESM(const ParamType *esm_params,const AMParams *am_params,const SSMParams *ssm_params) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using Difference of Jacobians\n");
	printf("Using %s order Current Self Hessian\n", hess_order.c_str());
	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::CurrentSelf, JacT::DiffOfJacs> ::updateHessian() {
	if(params.sec_ord_hess){
		am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
		record_event("am.cmptSelfHessian (second order)");
	} else{
		am.cmptSelfHessian(hessian, curr_pix_jacobian);
		record_event("am.cmptSelfHessian (first order)");
	}
}

template<class AM, class SSM>
FESM<AM, SSM, HessT::Std, JacT::DiffOfJacs> ::
FESM(const ParamType *esm_params,const AMParams *am_params,const SSMParams *ssm_params) :
	FESMBase< AM, SSM >(esm_params, am_params, ssm_params){
	printf("Using Difference of Jacobians\n");
	printf("Using %s order Standard Hessian\n", hess_order.c_str());
	curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPixCount());
}
template<class AM, class SSM>
void FESM<AM, SSM, HessT::Std, JacT::DiffOfJacs> ::updateHessian() {
	if(params.sec_ord_hess){
		am.cmptCurrHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
		record_event("am.cmptCurrHessian (second order)");
	} else{
		am.cmptCurrHessian(hessian, curr_pix_jacobian);
		record_event("am.cmptCurrHessian (first order)");
	}
}

_MTF_END_NAMESPACE

#include "mtf/Macros/register.h"

_REGISTER_ESM(FESMParams::HessType::Original, FESMParams::JacType::Original);
_REGISTER_ESM(FESMParams::HessType::SumOfStd, FESMParams::JacType::Original);
_REGISTER_ESM(FESMParams::HessType::SumOfSelf, FESMParams::JacType::Original);
_REGISTER_ESM(FESMParams::HessType::InitialSelf, FESMParams::JacType::Original);
_REGISTER_ESM(FESMParams::HessType::CurrentSelf, FESMParams::JacType::Original);
_REGISTER_ESM(FESMParams::HessType::Std, FESMParams::JacType::Original);

_REGISTER_ESM(FESMParams::HessType::Original, FESMParams::JacType::DiffOfJacs);
_REGISTER_ESM(FESMParams::HessType::SumOfStd, FESMParams::JacType::DiffOfJacs);
_REGISTER_ESM(FESMParams::HessType::SumOfSelf, FESMParams::JacType::DiffOfJacs);
_REGISTER_ESM(FESMParams::HessType::InitialSelf, FESMParams::JacType::DiffOfJacs);
_REGISTER_ESM(FESMParams::HessType::CurrentSelf, FESMParams::JacType::DiffOfJacs);
_REGISTER_ESM(FESMParams::HessType::Std, FESMParams::JacType::DiffOfJacs);