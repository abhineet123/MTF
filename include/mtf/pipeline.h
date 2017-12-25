#ifndef MTF_PIPELINE_H
#define MTF_PIPELINE_H

//! parameters for various components of the pipeline
#include "mtf/Config/parameters.h"
// tools for capturing images from disk or cameras
#include "mtf/Utilities/inputUtils.h"
//! tools for preprocessing the image
#include "mtf/Utilities/preprocUtils.h"
//! general OpenCV tools for selecting objects, reading ground truth, etc.
#include "mtf/Utilities/objUtils.h"
#include "mtf/Utilities/excpUtils.h"

#include <memory>

#ifndef HETEROGENEOUS_INPUT
#define HETEROGENEOUS_INPUT -1
#endif

#define OPENCV_PIPELINE 'c'
#ifndef DISABLE_XVISION
#define XVISION_PIPELINE 'x'
#endif
#ifndef DISABLE_VISP
#define VISP_PIPELINE 'v'
#endif

typedef std::unique_ptr<mtf::utils::InputBase> Input_;
typedef mtf::utils::PreProcBase::Ptr PreProc_;

_MTF_BEGIN_NAMESPACE

using namespace params;

inline utils::InputBase* getInput(char _pipeline_type){
	utils::InputParams _base_params(img_source, seq_name, seq_fmt, seq_path,
		input_buffer_size, invert_seq);
	switch(_pipeline_type){
	case OPENCV_PIPELINE:
	{
		utils::InputCVParams _params(&_base_params);
		return new utils::InputCV(&_params);
	}
#ifndef DISABLE_XVISION
	case XVISION_PIPELINE:
		return new utils::InputXV(&_base_params);
#endif
#ifndef DISABLE_VISP
	case VISP_PIPELINE:
	{
		utils::InputVPParams _params(&_base_params,
			vp_usb_n_buffers,
			static_cast<utils::InputVPParams::VpResUSB>(vp_usb_res),
			static_cast<utils::InputVPParams::VpFpsUSB>(vp_usb_fps),
			static_cast<utils::InputVPParams::VpResFW>(vp_fw_res),
			static_cast<utils::InputVPParams::VpFpsFW>(vp_fw_fps),
			static_cast<utils::InputVPParams::VpDepthPGFW>(vp_pg_fw_depth),
			vp_pg_fw_print_info,
			vp_pg_fw_shutter_ms,
			vp_pg_fw_gain,
			vp_pg_fw_exposure,
			vp_pg_fw_brightness
			);
		return new utils::InputVP(&_params);
	}
#endif
	default:
		throw utils::InvalidArgument(
			cv::format("Invalid pipeline type specified: %c\n",	_pipeline_type));
	}
}

inline utils::PreProcBase* createPreProc(int output_type, const std::string &_pre_proc_type){
	int _pre_proc_type_num = atoi(_pre_proc_type.c_str());
	if(_pre_proc_type_num < 0 || _pre_proc_type == "raw"){
		return new utils::NoPreProcessing(output_type);
	} else if(_pre_proc_type_num == 0 || _pre_proc_type == "none"){
		return new utils::NoFiltering(output_type, img_resize_factor, pre_proc_hist_eq);
	} else if(_pre_proc_type_num == 1 || _pre_proc_type == "gauss"){
		return new utils::GaussianSmoothing(output_type, img_resize_factor, pre_proc_hist_eq,
			gauss_kernel_size, gauss_sigma_x, gauss_sigma_y);
	} else if(_pre_proc_type_num == 2 || _pre_proc_type == "med"){
		return new utils::MedianFiltering(output_type, img_resize_factor, pre_proc_hist_eq, med_kernel_size);
	} else if(_pre_proc_type_num == 3 || _pre_proc_type == "box"){
		return new utils::NormalizedBoxFltering(output_type, img_resize_factor, pre_proc_hist_eq, box_kernel_size);
	} else if(_pre_proc_type_num == 4 || _pre_proc_type == "bil"){
		return new utils::BilateralFiltering(output_type, img_resize_factor, pre_proc_hist_eq,
			bil_diameter, bil_sigma_col, bil_sigma_space);
	} else if(_pre_proc_type_num == 5 || _pre_proc_type == "aniso"){
		return new utils::AnisotropicDiffusion(output_type, img_resize_factor, pre_proc_hist_eq,
			aniso_lambda, aniso_kappa, aniso_n_iters);
	} else if(_pre_proc_type_num == 6 || _pre_proc_type == "sobel"){
		return new utils::SobelFltering(output_type, img_resize_factor, pre_proc_hist_eq,
			sobel_kernel_size, sobel_normalize);
	} else{
		throw utils::InvalidArgument(
			cv::format("Invalid image pre processing type specified: %s\n",
			_pre_proc_type.c_str()));
	}
}

inline PreProc_ getPreProc(const vector<PreProc_> &existing_objs, int output_type,
	const std::string &_pre_proc_type){
	if(output_type == HETEROGENEOUS_INPUT){
		if(atoi(_pre_proc_type.c_str()) < 0 || _pre_proc_type == "raw"){
			throw utils::InvalidArgument(
				"getPreProc : Heterogeneos output cannot be used without pre processing");
		}
		PreProc_ new_obj = getPreProc(existing_objs, utils::supported_output_types[0], _pre_proc_type);
		PreProc_ curr_obj = new_obj;
		for(unsigned int output_id = 1; output_id < utils::supported_output_types.size(); ++output_id){
			curr_obj->next = getPreProc(existing_objs, utils::supported_output_types[output_id], _pre_proc_type);
			curr_obj = curr_obj->next;
		}
		return new_obj;
	}
	for(unsigned int obj_id = 0; obj_id < existing_objs.size(); ++obj_id){
		for(PreProc_ curr_obj = existing_objs[obj_id]; curr_obj; curr_obj = curr_obj->next){
			if(curr_obj->outputType() == output_type){ return curr_obj; }
		}
	}
	return PreProc_(createPreProc(output_type, _pre_proc_type));
}
inline PreProc_ getPreProc(int output_type,
	const std::string &_pre_proc_type){
	if(output_type == HETEROGENEOUS_INPUT){
		if(atoi(_pre_proc_type.c_str()) < 0 || _pre_proc_type == "raw"){
			throw utils::InvalidArgument(
				"getPreProc : Heterogeneos output cannot be used without pre processing");
		}
		PreProc_ new_obj = getPreProc(utils::supported_output_types[0], _pre_proc_type);
		PreProc_ curr_obj = new_obj;
		for(unsigned int output_id = 1; output_id < utils::supported_output_types.size(); ++output_id){
			curr_obj->next = getPreProc(utils::supported_output_types[output_id], _pre_proc_type);
			curr_obj = curr_obj->next;
		}
		return new_obj;
	}
	return PreProc_(createPreProc(output_type, _pre_proc_type));

}

inline bool getObjectsToTrack(utils::ObjUtils &obj_utils, utils::InputBase *input){
	bool init_obj_read = false;
	int n_objs_to_get = track_single_obj ? 1 : n_trackers;
	if(read_obj_from_gt){
		init_obj_read = obj_utils.readObjectFromGT(seq_name, seq_path, input->getNFrames(),
			init_frame_id, use_opt_gt, opt_gt_ssm, use_reinit_gt, invert_seq, debug_mode);
		if(!init_obj_read){
			printf("Failed to read initial object from ground truth; using manual selection...\n");
			read_obj_from_gt = 0;
		}
	}
	if(!init_obj_read && read_obj_from_file) {
		init_obj_read = obj_utils.readObjectsFromFile(n_objs_to_get, read_obj_fname.c_str(), debug_mode);
		if(!init_obj_read){
			printf("Failed to read initial object location from file; using manual selection...\n");
		}
	}
	if(!init_obj_read){
		if(img_source == SRC_IMG || img_source == SRC_DISK || img_source == SRC_VID){
			init_obj_read = obj_utils.selectObjects(input->getFrame(), n_objs_to_get,
				patch_size, line_thickness, write_objs, sel_quad_obj, write_obj_fname.c_str());
		} else{
			init_obj_read = obj_utils.selectObjects(input, n_objs_to_get,
				patch_size, line_thickness, write_objs, sel_quad_obj, write_obj_fname.c_str());
		}
	}
	return init_obj_read;
}

_MTF_END_NAMESPACE

#endif