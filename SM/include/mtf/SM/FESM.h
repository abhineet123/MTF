#ifndef MTF_FESM_H
#define MTF_FESM_H

#include "FESMBase.h"

#define HessT FESMParams::HessType
#define JacT FESMParams::JacType


#define get_base_members() \
	public: \
		using FESMBase<AM, SSM> ::am; \
		using FESMBase<AM, SSM> ::ssm; \
		using FESMBase<AM, SSM> ::name; \
		using FESMBase<AM, SSM> ::hess_order; \
		using typename FESMBase<AM, SSM> ::ParamType; \
		using typename FESMBase<AM, SSM> ::AMParams; \
		using typename FESMBase<AM, SSM> ::SSMParams; \
		using FESMBase<AM, SSM> ::params; \
		using FESMBase<AM, SSM> ::jacobian; \
		using FESMBase<AM, SSM> ::hessian; \
		using FESMBase<AM, SSM> ::init_pix_jacobian; \
		using FESMBase<AM, SSM> ::curr_pix_jacobian; \
		using FESMBase<AM, SSM> ::mean_pix_jacobian; \
		using FESMBase<AM, SSM> ::mean_pix_hessian; \
		using FESMBase<AM, SSM> ::init_pix_hessian; \
		using FESMBase<AM, SSM> ::curr_pix_hessian; \
		using FESMBase<AM, SSM> ::updatePixHessian; \
		FESM(const ParamType *esm_params = nullptr,const AMParams *am_params = nullptr,const SSMParams *ssm_params = nullptr)


_MTF_BEGIN_NAMESPACE

template<class AM, class SSM,
	HessT HT, JacT JT>
class FESM : public FESMBase < AM, SSM > {};

// ----------------------------------------------------------------------------------------//
// ---------------------------------- JacT::Original ------------------------------------- //
// ----------------------------------------------------------------------------------------//

template<class AM, class SSM >
class FESM<AM, SSM, HessT::Original, JacT::Original> :
	public FESMBase < AM, SSM >{
	get_base_members();	
};

template<class AM, class SSM>
class FESM<AM, SSM, HessT::SumOfStd, JacT::Original> :
	public FESMBase < AM, SSM >{
	get_base_members();
	
	void updateHessian() override;
};
template<class AM, class SSM>
class FESM<AM, SSM, HessT::SumOfSelf, JacT::Original> :
	public FESMBase < AM, SSM >{
	get_base_members();
	using FESMBase<AM, SSM> ::init_self_hessian;
	void initializeHessian() override;
	void updateHessian() override;
};
template<class AM, class SSM>
class FESM<AM, SSM, HessT::InitialSelf, JacT::Original> :
	public FESMBase < AM, SSM > {
	get_base_members();
	void initializeHessian() override;
	void updateHessian() override  {}
};

template<class AM, class SSM>
class FESM<AM, SSM, HessT::CurrentSelf, JacT::Original> :
	public FESMBase < AM, SSM >{
	get_base_members();
	void updateHessian() override;
};

template<class AM, class SSM>
class FESM<AM, SSM, HessT::Std, JacT::Original> :
	public FESMBase < AM, SSM >{
	get_base_members();
	void updateHessian() override;
};

// ----------------------------------------------------------------------------------------//
// ---------------------------------- JacT::DiffOfJacs ---------------------------------- //
// ----------------------------------------------------------------------------------------//

template<class AM, class SSM >
class FESM<AM, SSM, HessT::Original, JacT::DiffOfJacs> :
	public FESMBase < AM, SSM >{
	get_base_members();
	void updateJacobian() override;
};

template<class AM, class SSM>
class FESM<AM, SSM, HessT::SumOfStd, JacT::DiffOfJacs> :
	public FESMBase < AM, SSM >{
	get_base_members();
	void updateJacobian() override;
	void updateHessian() override;
};
template<class AM, class SSM>
class FESM<AM, SSM, HessT::SumOfSelf, JacT::DiffOfJacs> :
	public FESMBase < AM, SSM >{
	get_base_members();
	using FESMBase<AM, SSM> ::init_self_hessian;
	void updateJacobian() override;
	void initializeHessian() override;
	void updateHessian() override;
};
template<class AM, class SSM>
class FESM<AM, SSM, HessT::InitialSelf, JacT::DiffOfJacs> :
	public FESMBase < AM, SSM >{
	get_base_members();
	void updateJacobian() override;
	void initializeHessian() override;
	void updateHessian() override  {}
};

template<class AM, class SSM>
class FESM<AM, SSM, HessT::CurrentSelf, JacT::DiffOfJacs> :
	public FESMBase < AM, SSM >{
	get_base_members();
	void updateJacobian() override;
	void updateHessian() override;
};

template<class AM, class SSM>
class FESM<AM, SSM, HessT::Std, JacT::DiffOfJacs> :
	public FESMBase < AM, SSM >{
	get_base_members();
	void updateJacobian() override;
	void updateHessian() override;
};

_MTF_END_NAMESPACE

#endif

