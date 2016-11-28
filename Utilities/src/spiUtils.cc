#include "mtf/Utilities/spiUtils.h"

_MTF_BEGIN_NAMESPACE
namespace utils{

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
	void getProd(RowVectorXd &df_dp, const bool *spi_mask,
		const RowVectorXd &df_dI, const MatrixXd &dI_dp,
		int n_pix, int n_channels){
		assert(dI_dp.rows() == n_pix*n_channels && dI_dp.rows() == df_dp.size());
		df_dp.setZero();
		int ch_pix_id = 0;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			if(!spi_mask[pix_id]){ continue; }
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
			if(!spi_mask[pix_id]){ continue; }
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

