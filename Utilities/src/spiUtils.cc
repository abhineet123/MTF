#include "mtf/Utilities/spiUtils.h"
#include "mtf/Utilities/excpUtils.h"

#define SPI_PIX_DIFF_THRESH 10

_MTF_BEGIN_NAMESPACE
namespace utils{
	namespace spi{
		PixDiff::PixDiff(VectorXb &_mask, const ParamsType &params) :
			Base(_mask), pix_diff_thresh(SPI_PIX_DIFF_THRESH){
			if(params.empty()){ return; }
			if(params.size() != 1){
				throw InvalidArgument("SPIPixDiff needs exactly one input argument");
			}
			try{
				pix_diff_thresh = boost::any_cast<double>(params[0]);
			} catch(const boost::bad_any_cast &){
				throw InvalidArgument("SPIPixDiff :: Invalid parameter type provided for pix_diff_thresh");
			}
		}
		void PixDiff::initialize(const PixValT &init_pix_vals){
			max_pix_diff = init_pix_vals.maxCoeff() - init_pix_vals.minCoeff();
			rel_pix_diff.resize(init_pix_vals.size());
		}
		void PixDiff::update(const PixValT &init_pix_vals, const PixValT &curr_pix_vals){
			rel_pix_diff = (init_pix_vals - curr_pix_vals) / max_pix_diff;
			mask = rel_pix_diff.cwiseAbs().array() < pix_diff_thresh;
		}
	}
	double getMean(const bool *spi_mask, const VectorXd &vec,
		int vec_size){
		double mean = 0;
		int valid_vec_size = 0;
		for(int vec_id = 0; vec_id < vec_size; vec_id++){
			if(!spi_mask[vec_id]){ continue; }
			++valid_vec_size;
			mean += vec[vec_id];
		}
		mean /= valid_vec_size;
		return mean;
	}
	void getMean(RowVectorXd &mean_vec, const bool *spi_mask,
		const MatrixXd &mat, int n_rows){
		assert(mat.rows() == n_rows && mat.cols() == mean_vec.size());
		mean_vec.setZero();
		int valid_vec_size = 0;
		for(int row_id = 0; row_id < n_rows; row_id++){
			if(!spi_mask[row_id]){ continue; }
			++valid_vec_size;
			mean_vec += mat.row(row_id);
		}
		mean_vec /= valid_vec_size;
	}
	void getMean(VectorXd &mean_vec, const bool *spi_mask,
		const MatrixXd &mat, int n_cols){
		assert(mat.cols() == n_cols && mat.rows() == mean_vec.size());

		mean_vec.setZero();
		int valid_vec_size = 0;
		for(int col_id = 0; col_id < n_cols; col_id++){
			if(!spi_mask[col_id]){ continue; }
			++valid_vec_size;
			mean_vec += mat.col(col_id);
		}
		mean_vec /= valid_vec_size;
	}
	void getProd(RowVectorXd &df_dp, const bool *spi_mask,
		const RowVectorXd &df_dI, const MatrixXd &dI_dp,
		int n_pix, int n_channels){
		assert(dI_dp.rows() == n_pix*n_channels && dI_dp.rows() == df_dp.size());
		df_dp.setZero();
		int ch_pix_id = 0;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			if(!spi_mask[pix_id]){ ch_pix_id += n_channels;  continue; }
			for(int channel_id = 0; channel_id < n_channels; ++channel_id){
				df_dp += df_dI[ch_pix_id] * dI_dp.row(ch_pix_id);
				++ch_pix_id;
			}
		}
	}
	void getDiffOfProd(RowVectorXd &df_dp, const bool *spi_mask,
		const RowVectorXd &df_dIt, const MatrixXd &dIt_dp,
		const RowVectorXd &df_dI0, const MatrixXd &dI0_dp,
		int n_pix, int n_channels){
		assert(dIt_dp.rows() == n_pix*n_channels && dIt_dp.rows() == df_dp.size());
		assert(dI0_dp.rows() == n_pix*n_channels && dI0_dp.rows() == df_dp.size());
		df_dp.setZero();
		int ch_pix_id = 0;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			if(!spi_mask[pix_id]){ ch_pix_id += n_channels;  continue; }
			for(int channel_id = 0; channel_id < n_channels; ++channel_id){
				df_dp += df_dIt[ch_pix_id] * dIt_dp.row(ch_pix_id) -
					df_dI0[ch_pix_id] * dI0_dp.row(ch_pix_id);
				++ch_pix_id;
			}
		}
	}
	void expandMask(bool *out_mask, const bool *in_mask, int res_ratio_x,
		int res_ratio_y, int in_resx, int in_resy, int out_resx, int out_resy){
		assert(out_resx == in_resx*res_ratio_x);
		assert(out_resy == in_resy*res_ratio_y);
		for(int y_id = 0; y_id < in_resy; y_id++){
			int out_start_y = y_id*res_ratio_y;
			for(int x_id = 0; x_id < in_resx; x_id++){
				int in_id = y_id*in_resx + x_id;
				int out_start_x = x_id*res_ratio_x;
				for(int y = 0; y < res_ratio_y; y++){
					for(int x = 0; x < res_ratio_x; x++){
						int out_id = (out_start_y + y)*out_resx + (out_start_x + x);
						out_mask[out_id] = in_mask[in_id];
					}
				}
			}
		}
	}

}
_MTF_END_NAMESPACE

