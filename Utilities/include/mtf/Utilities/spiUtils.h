#ifndef MTF_SPI_UTILS_H
#define MTF_SPI_UTILS_H

#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE
namespace utils{
	double getMean(const bool *spi_mask, const VectorXd &vec,
		int vec_size);
	void getProd(RowVectorXd &df_dp, const bool *spi_mask,
		const RowVectorXd &df_dI, const MatrixXd &dI_dp,
		int n_pix, int n_channels);
	void getDiffOfProd(RowVectorXd &df_dp, const bool *spi_mask,
		const RowVectorXd &df_dIt, const MatrixXd &dIt_dp,
		const RowVectorXd &df_dI0, const MatrixXd &dI0_dp,		
		int n_pix, int n_channels);
	void expandMask(bool *out_mask, const bool *in_mask, int res_ratio_x,
		int res_ratio_y, int in_resx, int in_resy, int out_resx, int out_resy);

}
_MTF_END_NAMESPACE

#endif
