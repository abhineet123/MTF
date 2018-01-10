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
		return new utils::InputCV(&_base_params);
	}
#ifndef DISABLE_XVISION
	case XVISION_PIPELINE:
		return new utils::InputXV(&_base_params);
#endif
#ifndef DISABLE_VISP
	case VISP_PIPELINE:
	{
		utils::InputVPParams::VpResUSB usb_res;
		if(vp_usb_res == "default" || vp_usb_res == "0"){
			usb_res = utils::InputVPParams::VpResUSB::Default;
		} else if(vp_usb_res == "640x480" || vp_usb_res == "1"){
			usb_res = utils::InputVPParams::VpResUSB::res640x480;
		} else if(vp_usb_res == "800x600" || vp_usb_res == "2"){
			usb_res = utils::InputVPParams::VpResUSB::res800x600;
		} else if(vp_usb_res == "1024x768" || vp_usb_res == "3"){
			usb_res = utils::InputVPParams::VpResUSB::res1024x768;
		} else if(vp_usb_res == "1280x720" || vp_usb_res == "4"){
			usb_res = utils::InputVPParams::VpResUSB::res1280x720;
		} else if(vp_usb_res == "1920x1080" || vp_usb_res == "5"){
			usb_res = utils::InputVPParams::VpResUSB::res1920x1080;
		} else{
			throw utils::InvalidArgument(
				cv::format("Invalid resolution mode specified for ViSP USB pipeline: %s\n",
				vp_usb_res.c_str()));
		}
		utils::InputVPParams::VpFpsUSB usb_fps;
		if(vp_usb_fps == "default" || vp_usb_fps == "0"){
			usb_fps = utils::InputVPParams::VpFpsUSB::Default;
		} else if(vp_usb_fps == "25" || vp_usb_fps == "1"){
			usb_fps = utils::InputVPParams::VpFpsUSB::fps25;
		} else if(vp_usb_fps == "50" || vp_usb_fps == "2"){
			usb_fps = utils::InputVPParams::VpFpsUSB::fps50;
		} else{
			throw utils::InvalidArgument(
				cv::format("Invalid FPS mode specified for ViSP USB pipeline: %s\n",
				vp_usb_fps.c_str()));
		}
		utils::InputVPParams::VpResFW fw_res;
		if(vp_fw_res == "default" || vp_fw_res == "0"){
			fw_res = utils::InputVPParams::VpResFW::Default;
		} else if(vp_fw_res == "640x480" || vp_fw_res == "1"){
			fw_res = utils::InputVPParams::VpResFW::res640x480;
		} else if(vp_fw_res == "800x600" || vp_fw_res == "2"){
			fw_res = utils::InputVPParams::VpResFW::res800x600;
		} else if(vp_fw_res == "1024x768" || vp_fw_res == "3"){
			fw_res = utils::InputVPParams::VpResFW::res1024x768;
		} else if(vp_fw_res == "1280x960" || vp_fw_res == "4"){
			fw_res = utils::InputVPParams::VpResFW::res1280x960;
		} else if(vp_fw_res == "1600x1200" || vp_fw_res == "5"){
			fw_res = utils::InputVPParams::VpResFW::res1600x1200;
		} else{
			throw utils::InvalidArgument(
				cv::format("Invalid resolution mode specified for ViSP firewire pipeline: %s\n",
				vp_fw_res.c_str()));
		}
		utils::InputVPParams::VpFpsFW fw_fps;
		if(vp_fw_fps == "default" || vp_fw_fps == "0"){
			fw_fps = utils::InputVPParams::VpFpsFW::Default;
		} else if(vp_fw_fps == "15" || vp_fw_fps == "1"){
			fw_fps = utils::InputVPParams::VpFpsFW::fps15;
		} else if(vp_fw_fps == "30" || vp_fw_fps == "2"){
			fw_fps = utils::InputVPParams::VpFpsFW::fps30;
		} else if(vp_fw_fps == "60" || vp_fw_fps == "3"){
			fw_fps = utils::InputVPParams::VpFpsFW::fps60;
		} else if(vp_fw_fps == "120" || vp_fw_fps == "4"){
			fw_fps = utils::InputVPParams::VpFpsFW::fps120;
		} else if(vp_fw_fps == "240" || vp_fw_fps == "5"){
			fw_fps = utils::InputVPParams::VpFpsFW::fps240;
		}  else if(vp_fw_fps == "7.5" || vp_fw_fps == "6"){
			fw_fps = utils::InputVPParams::VpFpsFW::fps7_5;
		}  else if(vp_fw_fps == "3.75" || vp_fw_fps == "7"){
			fw_fps = utils::InputVPParams::VpFpsFW::fps3_75;
		}  else if(vp_fw_fps == "1.875" || vp_fw_fps == "8"){
			fw_fps = utils::InputVPParams::VpFpsFW::fps1_875;
		} else{
			throw utils::InvalidArgument(
				cv::format("Invalid FPS mode specified for ViSP firewire pipeline: %s\n",
				vp_fw_fps.c_str()));
		}
		utils::InputVPParams::VpDepthFW fw_depth;
		if(vp_fw_depth == "default" || vp_fw_depth == "0"){
			fw_depth = utils::InputVPParams::VpDepthFW::Default;
		} else if(vp_fw_depth == "rgb" || vp_fw_depth == "1"){
			fw_depth = utils::InputVPParams::VpDepthFW::RGB;
		} else if(vp_fw_depth == "yuv422" || vp_fw_depth == "2"){
			fw_depth = utils::InputVPParams::VpDepthFW::YUV422;
		} else if(vp_fw_depth == "mono8" || vp_fw_depth == "3"){
			fw_depth = utils::InputVPParams::VpDepthFW::MONO8;
		} else if(vp_fw_depth == "mono16" || vp_fw_depth == "4"){
			fw_depth = utils::InputVPParams::VpDepthFW::MONO16;
		} else{
			throw utils::InvalidArgument(
				cv::format("Invalid image depth mode specified for ViSP firewire pipeline: %s\n",
				vp_fw_depth.c_str()));
		}
		utils::InputVPParams::VpISOFW fw_iso;
		if(vp_fw_iso == "default" || vp_fw_iso == "0"){
			fw_iso = utils::InputVPParams::VpISOFW::Default;
		} else if(vp_fw_iso == "100" || vp_fw_iso == "1"){
			fw_iso = utils::InputVPParams::VpISOFW::iso100;
		} else if(vp_fw_iso == "200" || vp_fw_iso == "2"){
			fw_iso = utils::InputVPParams::VpISOFW::iso200;
		} else if(vp_fw_iso == "400" || vp_fw_iso == "3"){
			fw_iso = utils::InputVPParams::VpISOFW::iso400;
		} else if(vp_fw_iso == "800" || vp_fw_iso == "4"){
			fw_iso = utils::InputVPParams::VpISOFW::iso800;
		} else if(vp_fw_iso == "1600" || vp_fw_iso == "5"){
			fw_iso = utils::InputVPParams::VpISOFW::iso1600;
		} else if(vp_fw_iso == "3200" || vp_fw_iso == "6"){
			fw_iso = utils::InputVPParams::VpISOFW::iso3200;
		} else{
			throw utils::InvalidArgument(
				cv::format("Invalid ISO mode specified for ViSP firewire pipeline: %s\n",
				vp_fw_iso.c_str()));
		}
		utils::InputVPParams _params(&_base_params,
			vp_usb_n_buffers,
			usb_res, usb_fps,
			fw_res, fw_fps, fw_depth, fw_iso,
			vp_fw_print_info,
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
		return new utils::NormalizedBoxFiltering(output_type, img_resize_factor, pre_proc_hist_eq, box_kernel_size);
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