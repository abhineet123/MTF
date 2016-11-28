#ifndef MTF_AESM_NT_H
#define MTF_AESM_NT_H

#include "ESM.h"

_MTF_BEGIN_NAMESPACE
namespace nt{
	
// Additive formulation of ESM
class AESM : public ESM {

	inherit_profiling(ESM);

public:
	using ESM ::am;
	using ESM ::ssm;
	using ESM ::ParamType;
	using ESM ::cv_corners_mat;
	using ESM ::name;
	using ESM ::initialize;
	using ESM ::update;

	using ESM ::params;
	using ESM ::log_fname;
	using ESM ::time_fname;
	using ESM ::init_pix_jacobian;
	using ESM ::curr_pix_jacobian;
	using ESM ::init_pix_hessian;
	using ESM ::curr_pix_hessian;
	using ESM ::ssm_update;

	void initializePixJacobian() override;
	void updatePixJacobian() override;
	void initializePixHessian() override;
	void updatePixHessian() override;
	void updateState() override;

	AESM(AM _am, SSM _ssm, const ParamType *aesm_params = nullptr);
};
}

_MTF_END_NAMESPACE

#endif

