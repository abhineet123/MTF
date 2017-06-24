#ifndef MTF_SPI_UTILS_H
#define MTF_SPI_UTILS_H

#include "mtf/Macros/common.h"

#include <boost/any.hpp>
#include <vector>

_MTF_BEGIN_NAMESPACE
namespace utils{
	namespace spi{
		enum class Types{ None, PixDiff, Gradient, GFTT };
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
			double pix_diff_thresh;
			VectorXd rel_pix_diff;
			double max_pix_diff;
		};
		struct Gradient : Base{
			Gradient(VectorXb &mask, const ParamsType &params);
			void initialize(const PixGradT &init_pix_grad);
			void update(const PixGradT &curr_pix_grad);
		private:
			double grad_thresh;
			bool use_union;
			unsigned int n_pix;
			VectorXd pix_grad_norm;
			VectorXb init_mask;
		};
		//! Good Features To Track
		struct GFTT : Base{
			GFTT(VectorXb &mask, const ParamsType &params);
			void initialize(const PixValT &init_pix_vals, 
				unsigned int _resx, unsigned int _resy);
			void update(const PixValT &curr_pix_vals);
		private:
			int max_corners;
			double quality_level;
			double min_distance;
			int block_size;
			bool use_harris_detector;
			double k;
			bool use_union;
			int neigh_offset;
			unsigned int resx, resy, n_pix;
			cv::Mat curr_patch_32f, good_locations;
			VectorXb init_mask;
			void getMask(const PixValT &curr_pix_vals, VectorXb &init_spi_mask);
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
