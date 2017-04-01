#ifndef MTF_MC_LSCV_H
#define MTF_MC_LSCV_H

#include "LSCV.h"

_MTF_BEGIN_NAMESPACE

class MCLSCV : public LSCV{
public:
	typedef Map<MatrixXdr, Unaligned, InnerStride<3> > MatrixXdMrMC;
	typedef Map<VectorXd, Unaligned, InnerStride<3> > VectorXdMMC;

	MCLSCV(const ParamType *scv_params = nullptr);

	void initializePixVals(const Matrix2Xd& init_pts) override;
	void updatePixVals(const Matrix2Xd& curr_pts) override;
	void updateSimilarity(bool prereq_only = true) override;

protected:

	void updateMappedPixVals(int index, int ch);
};

_MTF_END_NAMESPACE

#endif