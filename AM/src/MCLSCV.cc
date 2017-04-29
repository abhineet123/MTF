#include "mtf/AM/MCLSCV.h"
#include "mtf/Utilities/imgUtils.h"

_MTF_BEGIN_NAMESPACE

MCLSCV::MCLSCV(const ParamType *lscv_params) : 
LSCV(lscv_params, 3){
	printf("Using Multi Channel variant\n");
	name = "mclscv";
}


void MCLSCV::initializePixVals(const Matrix2Xd& init_pts){
	if(!is_initialized.pix_vals){
		I0.resize(patch_size);
		It.resize(patch_size);
	}
	if(params.uchar_input){
		utils::mc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	} else{
		utils::mc::getPixVals<float>(I0, curr_img_cv, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	}
	/**
	create a copy of the initial pixel values which will hold the original untampered template
	that will be used for remapping the initial pixel values when the intensity map is updated;
	it will also be used for updating the intensity map itself since using the remapped
	initial pixel values to update the joint probability distribution and thus the intensity map will cause bias
	*/
	I0_orig = I0;

	if(!is_initialized.pix_vals){
		It = I0;
		is_initialized.pix_vals = true;
	}
}

void MCLSCV::updatePixVals(const Matrix2Xd& curr_pts){
	if(params.uchar_input){
		utils::mc::getPixVals<uchar>(It, curr_img_cv, curr_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	} else{
		utils::mc::getPixVals<float>(It, curr_img_cv, curr_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	}	
}

void MCLSCV::updateSimilarity(bool prereq_only){
	if(params.once_per_frame && !first_iter)
		return;
	//! channel wise processing
	for(int ch = 0; ch < 3; ch++){
		for(int idx = 0; idx < params.n_sub_regions_x; idx++){
			int start_x = sub_region_x(idx, 0);
			int end_x = sub_region_x(idx, 1);
			for(int idy = 0; idy < params.n_sub_regions_y; idy++){
				curr_hist.fill(hist_pre_seed);
				init_hist.fill(hist_pre_seed);
				curr_joint_hist.fill(0);
				for(int y = sub_region_y(idy, 0); y <= sub_region_y(idy, 1); y++) {
					for(int x = start_x; x <= end_x; x++) {
						int patch_id = (y*resx + x) * 3 + ch;
						int pix1_int = static_cast<int>(It(patch_id));
						int pix2_int = static_cast<int>(I0_orig(patch_id));
						//if(pix1_int>255 || pix1_int<0){
						//	printf("pix2_int: %d It(%d): %f\n", pix1_int, patch_id, It(patch_id));
						//	return;
						//}
						//if(pix2_int>255 || pix2_int<0){
						//	printf("pix2_int: %d I0_orig(%d): %f\n", pix2_int, patch_id, I0_orig(patch_id));
						//	return;
						//}
						curr_hist(pix1_int) += 1;
						init_hist(pix2_int) += 1;
						curr_joint_hist(pix1_int, pix2_int) += 1;
					}
				}
				for(int bin_id = 0; bin_id < params.n_bins; ++bin_id){
					if(init_hist(bin_id) == 0){
						intensity_map(bin_id) = bin_id;
					} else{
						double wt_sum = 0;
						for(int i = 0; i < params.n_bins; ++i){
							wt_sum += i * curr_joint_hist(i, bin_id);
						}
						intensity_map(bin_id) = wt_sum / init_hist(bin_id);
					}
				}
				updateMappedPixVals(_subregion_idx(idy, idx), ch);
				for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id){
					I0(pix_id * 3 + ch) += I0_mapped(pix_id)*sub_region_wts(pix_id, _subregion_idx(idy, idx));
				}
			}
		}
	}
}
void MCLSCV::updateMappedPixVals(int index, int ch){
	VectorXdMMC I0_orig_ch(I0_orig.data() + ch, n_pix);
	if(params.affine_mapping){
		Vector2d affine_params = intensity_vals_dec.solve(intensity_map);
		I0_mapped = (affine_params(0)*I0_orig_ch).array() + affine_params(1);
	} else{
		if(params.weighted_mapping){
			for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id){
				I0_mapped(pix_id) = utils::mapPixVal<utils::InterpType::Linear>(I0_orig_ch(pix_id), intensity_map);
			}
		} else{
			for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id){
				I0_mapped(pix_id) = utils::mapPixVal<utils::InterpType::Nearest>(I0_orig_ch(pix_id), intensity_map);
			}
		}
	}
}

_MTF_END_NAMESPACE

