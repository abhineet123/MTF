#ifndef MTF_SPI_UTILS_H
#define MTF_SPI_UTILS_H

#include "mtf/Macros/common.h"

#include <boost/any.hpp>
#include <vector>

_MTF_BEGIN_NAMESPACE
namespace utils{
	namespace spi{
		typedef std::vector<boost::any> ParamsType;
		struct Base{
			Base(VectorXb &_mask) :mask(_mask){}
		protected:
			VectorXb &mask;
		};
		struct PixDiff : Base{
			PixDiff(VectorXb &mask, const ParamsType &params);
			void initialize(const PixValT &init_pix_vals);
			void update(const PixValT &init_pix_vals, const PixValT &curr_pix_vals);
		private:
			VectorXb &mask;
			double pix_diff_thresh;
			VectorXd rel_pix_diff;
			double max_pix_diff;
		};
	}
	double getMean(const bool *spi_mask, const VectorXd &vec,
		int vec_size);
	//! columnwise mean
	void getMean(RowVectorXd &mean_vec, const bool *spi_mask,
		const MatrixXd &mat, int n_rows);
	//! rowwise mean
	void getMean(VectorXd &mean_vec, const bool *spi_mask,
		const MatrixXd &mat, int n_cols);
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
