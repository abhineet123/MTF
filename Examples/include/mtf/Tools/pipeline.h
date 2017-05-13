#ifndef MTF_PIPELINE_H
#define MTF_PIPELINE_H

// tools for capturing images from disk or cameras
#include "inputCV.h"
#ifndef DISABLE_THIRD_PARTY_TRACKERS
#ifndef DISABLE_XVISION
#include "inputXV.h"
#endif
#ifndef DISABLE_VISP
#include "inputVP.h"
#endif
#endif
//! tools for preprocessing the image
#include "PreProc.h"
//! general OpenCV tools for selecting objects, reading ground truth, etc.
#include "cvUtils.h"
//! parameters for various components of the pipeline
#include "mtf/Config/parameters.h"
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

using namespace mtf::params;

typedef std::unique_ptr<InputBase> Input_;
typedef PreProcBase::Ptr PreProc_;

inline InputBase* getInput(char _pipeline_type){
	switch(_pipeline_type){
	case OPENCV_PIPELINE:
		return new InputCV(img_source, seq_name, seq_fmt, seq_path, input_buffer_size, invert_seq);
#ifndef DISABLE_XVISION
	case XVISION_PIPELINE:
		return new InputXV(img_source, seq_name, seq_fmt, seq_path, input_buffer_size, invert_seq);
#endif
#ifndef DISABLE_VISP
	case VISP_PIPELINE:
		return new InputVP(
			img_source, seq_name, seq_fmt, seq_path, input_buffer_size, visp_usb_n_buffers, invert_seq,
			static_cast<VpResUSB>(visp_usb_res), static_cast<VpFpsUSB>(visp_usb_fps),
			static_cast<VpResFW>(visp_fw_res), static_cast<VpFpsFW>(visp_fw_fps)
			);
#endif
	default:
		throw mtf::utils::InvalidArgument(
			cv::format("Invalid pipeline type specified: %c\n",	_pipeline_type));
	}
}

inline PreProcBase* createPreProc(int output_type, const std::string &_pre_proc_type){
	int _pre_proc_type_num = atoi(_pre_proc_type.c_str());
	if(_pre_proc_type_num < 0 || _pre_proc_type == "raw"){
		return new NoPreProcessing(output_type);
	} else if(_pre_proc_type_num == 0 || _pre_proc_type == "none"){
		return new NoFiltering(output_type, img_resize_factor, pre_proc_hist_eq);
	} else if(_pre_proc_type_num == 1 || _pre_proc_type == "gauss"){
		return new GaussianSmoothing(output_type, img_resize_factor, pre_proc_hist_eq,
			gauss_kernel_size, gauss_sigma_x, gauss_sigma_y);
	} else if(_pre_proc_type_num == 2 || _pre_proc_type == "med"){
		return new MedianFiltering(output_type, img_resize_factor, pre_proc_hist_eq, med_kernel_size);
	} else if(_pre_proc_type_num == 3 || _pre_proc_type == "box"){
		return new NormalizedBoxFltering(output_type, img_resize_factor, pre_proc_hist_eq, box_kernel_size);
	} else if(_pre_proc_type_num == 4 || _pre_proc_type == "bil"){
		return new BilateralFiltering(output_type, img_resize_factor, pre_proc_hist_eq,
			bil_diameter, bil_sigma_col, bil_sigma_space);
	} else{
		throw mtf::utils::InvalidArgument(
			cv::format("Invalid image pre processing type specified: %s\n",
			_pre_proc_type.c_str()));
	}
}

inline PreProc_ getPreProc(const vector<PreProc_> &existing_objs, int output_type,
	const std::string &_pre_proc_type){
	if(output_type == HETEROGENEOUS_INPUT){
		if(atoi(_pre_proc_type.c_str()) < 0 || _pre_proc_type == "raw"){
			throw mtf::utils::InvalidArgument(
				"getPreProc : Heterogeneos output cannot be used without pre processing");
		}
		PreProc_ new_obj = getPreProc(existing_objs, supported_output_types[0], _pre_proc_type);
		PreProc_ curr_obj = new_obj;
		for(unsigned int output_id = 1; output_id < supported_output_types.size(); ++output_id){
			curr_obj->next = getPreProc(existing_objs, supported_output_types[output_id], _pre_proc_type);
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
			throw mtf::utils::InvalidArgument(
				"getPreProc : Heterogeneos output cannot be used without pre processing");
		}
		PreProc_ new_obj = getPreProc(supported_output_types[0], _pre_proc_type);
		PreProc_ curr_obj = new_obj;
		for(unsigned int output_id = 1; output_id < supported_output_types.size(); ++output_id){
			curr_obj->next = getPreProc(supported_output_types[output_id], _pre_proc_type);
			curr_obj = curr_obj->next;
		}
		return new_obj;
	}
	return PreProc_(createPreProc(output_type, _pre_proc_type));

}

inline bool getObjectsToTrack(CVUtils &cv_utils, InputBase *input){
	bool init_obj_read = false;
	int n_objs_to_get = track_single_obj ? 1 : n_trackers;
	if(read_obj_from_gt){
		init_obj_read = cv_utils.readObjectFromGT(seq_name, seq_path, input->n_frames,
			init_frame_id, use_opt_gt, opt_gt_ssm, use_reinit_gt, invert_seq, debug_mode);
		if(!init_obj_read){
			printf("Failed to read initial object from ground truth; using manual selection...\n");
			read_obj_from_gt = 0;
		}
	}
	if(!init_obj_read && read_obj_from_file) {
		init_obj_read = cv_utils.readObjectsFromFile(n_objs_to_get, read_obj_fname.c_str(), debug_mode);
		if(!init_obj_read){
			printf("Failed to read initial object location from file; using manual selection...\n");
		}
	}
	if(!init_obj_read){
		if(img_source == SRC_IMG || img_source == SRC_DISK || img_source == SRC_VID){
			init_obj_read = cv_utils.selectObjects(input->getFrame(), n_objs_to_get,
				patch_size, line_thickness, write_objs, sel_quad_obj, write_obj_fname.c_str());
		} else{
			init_obj_read = cv_utils.selectObjects(input, n_objs_to_get,
				patch_size, line_thickness, write_objs, sel_quad_obj, write_obj_fname.c_str());
		}
	}
	return init_obj_read;
}

#endif