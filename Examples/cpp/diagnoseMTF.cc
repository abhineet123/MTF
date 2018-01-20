#include "mtf/mtf.h"
#include "mtf/pipeline.h"
#include "mtf/Diagnostics/Diagnostics.h"
#include "mtf/Config/parameters.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/excpUtils.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include <time.h>
#include <numeric>
#include <vector>
#include <memory>
#include <fstream>

using namespace std;
namespace fs = boost::filesystem;
using namespace mtf::params;

typedef mtf::Diagnostics::AnalyticalDataType ADT;
typedef mtf::Diagnostics::NumericalDataType NDT;
typedef std::unique_ptr<mtf::Diagnostics> Diag;

VectorXd param_range;
VectorXd x_vec, y_vec;

Diag diag;

const char* getDataTypeName(int data_id, int adt_len, int diag_len);
void generateData(int data_id, int adt_len, int diag_len);
void generateInverseData(int data_id, int adt_len, int diag_len);

int main(int argc, char * argv[]) {

	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

	cout << "*******************************\n";
	cout << "Running MTF diagnostics with parameters:\n";
	cout << "source_id: " << seq_id << "\n";
	cout << "source_name: " << seq_name << "\n";
	cout << "actor: " << actor << "\n";
	cout << "pipeline: " << pipeline << "\n";
	cout << "img_source: " << img_source << "\n";
	cout << "mtf_visualize: " << mtf_visualize << "\n";
	cout << "read_obj_from_file: " << read_obj_from_file << "\n";
	cout << "patch_size: " << patch_size << "\n";
	cout << "read_obj_fname: " << read_obj_fname << "\n";
	cout << "read_obj_from_gt: " << read_obj_from_gt << "\n";
	cout << "pause_after_frame: " << pause_after_frame << "\n";
	cout << "*******************************\n";

	Input_ input(mtf::getInput(pipeline));
	if(!input->initialize()){
		printf("Pipeline could not be initialized successfully. Exiting...\n");
		return EXIT_FAILURE;
	}
	printf("n_frames: %d\n", input->getNFrames());

	for(int frame_id = 0; frame_id < diag_start_id; ++frame_id){
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline", input->getFrameID() + 1);
			return EXIT_FAILURE;
		}
	}

	mtf::utils::ObjUtils obj_utils;
	try{
		if(!mtf::getObjectsToTrack(obj_utils, input.get())){
			printf("Object(s) to be tracked could not be read\n");
			return EXIT_FAILURE;
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while obtaining the objects to track: %s\n",
			err.type(), err.what());
		return EXIT_FAILURE;
	}


	/*********************************** initialize trackers ***********************************/

	printf("Using diagnostics with object of size %f x %f\n",
		obj_utils.getObj(0).size_x, obj_utils.getObj(0).size_y);
	if(res_from_size){
		resx = static_cast<unsigned int>(res_from_size*obj_utils.getObj(0).size_x);
		resy = static_cast<unsigned int>(res_from_size*obj_utils.getObj(0).size_y);
	}
	mtf::AM am(mtf::getAM(diag_am, diag_ilm));
	mtf::SSM ssm(mtf::getSSM(diag_ssm));
	if(!am || !ssm){
		printf("Diagnostics could not be initialized successfully\n");
		return EXIT_FAILURE;
	}
	mtf::DiagnosticsParams diag_params(
		static_cast<mtf::DiagnosticsParams::UpdateType>(diag_update_type),
		diag_show_data, diag_show_corners, diag_show_patches,
		diag_enable_validation, diag_validation_prec);
	diag.reset(new  mtf::Diagnostics(am, ssm, &diag_params));

	/* initialize frame pre processor*/
	PreProc_ pre_proc = mtf::getPreProc(diag->inputType(), pre_proc_type);
	pre_proc->initialize(input->getFrame());
	//char feat_norm_fname[100], norm_fname[100], jac_fname[100], hess_fname[100], hess2_fname[100];
	//char jac_num_fname[100], hess_num_fname[100], nhess_num_fname[100];
	//char ssm_fname[100];
	std::string diag_data_dir = cv::format("log/diagnostics/%s", seq_name.c_str());
	if(diag_3d){
		diag_data_dir = diag_data_dir + "/3D";
	}
	if(!fs::exists(diag_data_dir)){
		printf("Diagnostic data directory: %s does not exist. Creating it...\n", diag_data_dir.c_str());
		fs::create_directories(diag_data_dir);
	}
	diag->setImage(pre_proc->getFrame());

	param_range.resize(diag->state_size);
	if(diag_range){
		param_range.fill(diag_range);
	} else{
		diag_ssm_range = ssm_sigma[diag_ssm_range_id];
		if(diag_ssm_range.size() < diag->ssm_state_size){
			throw mtf::utils::InvalidArgument("diagnoseMTF:: Insufficient number of SSM range parameters provided");
		}
		param_range.head(diag->ssm_state_size) = Map<const VectorXd>(diag_ssm_range.data(),
			diag->ssm_state_size);
		if(diag->am_state_size){
			diag_am_range = am_sigma[diag_am_range_id];
			if(diag_am_range.size() < diag->am_state_size){
				throw mtf::utils::InvalidArgument("diagnoseMTF:: Insufficient number of AM range parameters provided");
			}
			param_range.tail(diag->am_state_size) = Map<const VectorXd>(diag_am_range.data(),
				diag->am_state_size);
		}
	}
	mtf::utils::printMatrix(param_range.transpose(), "param_range");
	if(res_from_size){
		printf("Getting sampling resolution from object size...\n");
	}
	bool gen_flags[4];
	gen_flags[0] = diag_gen_jac.find('1') != string::npos;
	gen_flags[1] = diag_gen_hess.find('1') != string::npos ||
		diag_gen_hess_sum[0] == '1' || diag_gen_hess_sum[2] == '3';
	gen_flags[2] = diag_gen_hess2.find('1') != string::npos ||
		diag_gen_hess_sum[1] == '1' || diag_gen_hess_sum[3] == '3';
	gen_flags[3] = diag_gen_norm[1] == '1';

	diag->initialize(obj_utils.getObj(0).corners, gen_flags);


	std::string diag_gen = diag_gen_norm + diag_gen_jac + diag_gen_hess +
		diag_gen_hess2 + diag_gen_hess_sum + diag_gen_num + diag_gen_ssm;

	int diag_len = diag_gen.length();
	int adt_len = diag_gen_norm.length() + diag_gen_jac.length() +
		diag_gen_hess.length() + diag_gen_hess2.length() + diag_gen_hess_sum.length();

	printf("diag_gen_norm: %s\n", diag_gen_norm.c_str());
	printf("diag_gen_jac: %s\n", diag_gen_jac.c_str());
	printf("diag_gen_hess: %s\n", diag_gen_hess.c_str());
	printf("diag_gen_hess2: %s\n", diag_gen_hess2.c_str());
	printf("diag_gen_hess_sum: %s\n", diag_gen_hess_sum.c_str());
	printf("diag_gen_num: %s\n", diag_gen_num.c_str());
	printf("diag_gen_ssm: %s\n", diag_gen_ssm.c_str());
	printf("diag_gen: %s\n", diag_gen.c_str());
	printf("diag_len: %d\n", diag_len);
	printf("adt_len: %d\n", adt_len);
	printf("diag_frame_gap: %d\n", diag_frame_gap);

	int start_id = diag_start_id >= diag_frame_gap ? diag_start_id : diag_frame_gap;
	int end_id = diag_end_id >= diag_start_id ? diag_end_id : input->getNFrames() - 1;
	if(end_id > start_id){
		printf("Generating diagnostics data for frames %d to %d\n", start_id, end_id);
	} else {
		end_id = start_id;
		printf("Generating diagnostics data for frame %d\n", start_id);
	}
	if(diag_out_prefix.empty()){
		diag_out_prefix = cv::format("%s_%s", diag_am, diag_ssm);
		if(std::string(diag_ilm) != "0"){
			diag_out_prefix = cv::format("%s_%s", diag_out_prefix.c_str(), diag_ilm);
		}
	}
	std::string bin_out_fname;
	std::vector<std::shared_ptr<std::ofstream> > out_files;
	out_files.resize(diag_len);
	if(diag_bin){
		for(int data_id = 0; data_id < diag_len; data_id++){
			if(diag_gen[data_id] - '0'){
				const char* data_name = getDataTypeName(data_id, adt_len, diag_len);
				if(diag_inv){
					bin_out_fname = cv::format("%s/%s_%d_inv_%s_%d_%d_%d.bin",
						diag_data_dir.c_str(), diag_out_prefix.c_str(), diag_update_type, data_name, 
						diag_frame_gap, start_id, end_id);
				} else{
					bin_out_fname = cv::format("%s/%s_%d_%s_%d_%d_%d.bin",
						diag_data_dir.c_str(), diag_out_prefix.c_str(), diag_update_type, data_name,
						diag_frame_gap, start_id, end_id);
				}

				if(diag_inv){
					printf("Writing inv_%s data to %s\n", data_name, bin_out_fname.c_str());
				} else{
					printf("Writing %s data to %s\n", data_name, bin_out_fname.c_str());
				}
				out_files[data_id].reset(new std::ofstream(bin_out_fname, ios::out | ios::binary));
				if(diag_3d){
					Vector4i header(start_id - 1, start_id, diag_res, diag_res);
					out_files[data_id]->write((char*)(header.data()), sizeof(int) * 4);
				} else{
					out_files[data_id]->write((char*)(&diag_res), sizeof(int));
					out_files[data_id]->write((char*)(&(diag->state_size)), sizeof(int));
				}
			}
		}
	}

	for(int i = diag_start_id; i < start_id; i++){
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline", input->getFrameID() + 1);
			return EXIT_FAILURE;
		}
	}
	pre_proc->update(input->getFrame(), input->getFrameID());

	printf("frame_id: %d\n", input->getFrameID());
	vector<double> proc_times;
	double total_time_taken = 0;
	for(int frame_id = start_id; frame_id <= end_id; frame_id++){
		printf("Processing frame: %d\n", frame_id);
		//update diagnostics module
		diag->update(obj_utils.getGT(frame_id - diag_frame_gap));

		mtf_clock_get(frame_start_time);
		for(int data_id = 0; data_id < diag_len; data_id++){
			if(diag_gen[data_id] - '0'){
				mtf_clock_get(start_time);				
				if(diag_inv){
					generateInverseData(data_id, adt_len, diag_len);
				} else{
					generateData(data_id, adt_len, diag_len);
				}
				mtf_clock_get(end_time);
				
				double time_taken;
				mtf_clock_measure(start_time, end_time, time_taken);
				total_time_taken += time_taken;
				if(diag_verbose){
					printf("Time taken:\t %f\n", time_taken);
				}
				if(diag_3d){
					if(frame_id == start_id){
						out_files[data_id]->write((char*)(y_vec.data()), sizeof(double)*y_vec.size());
						out_files[data_id]->write((char*)(x_vec.data()), sizeof(double)*x_vec.size());
					}
					out_files[data_id]->write((char*)(diag->getData().data()),
						sizeof(double)*diag->getData().size());

					long curr_pos = static_cast<long>(out_files[data_id]->tellp());
					out_files[data_id]->seekp(0, ios_base::beg);
					out_files[data_id]->write((char*)(&frame_id), sizeof(int));
					out_files[data_id]->seekp(curr_pos, ios_base::beg);
				} else{
					out_files[data_id]->write((char*)(diag->getData().data()),
						sizeof(double)*diag->getData().size());
				}

			}
		}
		mtf_clock_get(frame_end_time);
		double frame_time_taken;
		mtf_clock_measure(frame_start_time, frame_end_time, frame_time_taken);
		proc_times.push_back(frame_time_taken);
		if(diag_verbose){
			printf("Done frame %d. Time taken:\t %f\n", frame_id, frame_time_taken);
		}
		if(diag_verbose){
			printf("***********************************\n");
		}

		if(input->getFrameID() == end_id){ break; }

		// update frame
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline", input->getFrameID() + 1);
			break;
		}
		pre_proc->update(input->getFrame(), input->getFrameID());
	}
	printf("Closing files...\n");
	if(diag_bin){
		for(int data_id = 0; data_id < diag_len; data_id++){
			if(diag_gen[data_id] - '0'){
				out_files[data_id]->write((char*)(proc_times.data()), sizeof(double)*proc_times.size());
				out_files[data_id]->close();
			}
		}
		out_files.clear();
	}
	if(diag_verbose){
		printf("Total Time taken:\t %f\n", total_time_taken);
	}
	printf("Done closing files\n");
	return EXIT_SUCCESS;
}

void generateData(int data_id, int adt_len, int diag_len){
	char *out_fname = nullptr;
	if(data_id < adt_len){// analytical
		if(diag_verbose){
			printf("Generating %s data\t", diag->toString(static_cast<ADT>(data_id)));
		}
		if(diag_3d){
			Vector2i state_ids(diag_3d_ids[0], diag_3d_ids[1]);
			diag->generateAnalyticalData3D(x_vec, y_vec, param_range, diag_res, state_ids,
				static_cast<ADT>(data_id), out_fname);
		} else{
			diag->generateAnalyticalData(param_range, diag_res, static_cast<ADT>(data_id), out_fname);
		}
	} else if(data_id < diag_len - 1){// numerical
		int num_data_id = data_id - adt_len;
		//printf("data_id: %d\n", data_id);
		//printf("adt_len: %d\n", adt_len);
		//printf("num_data_id: %d\n", num_data_id);
		if(diag_verbose){
			printf("Generating numerical %s data\t",
				diag->toString(static_cast<NDT>(num_data_id)));
		}
		diag->generateNumericalData(param_range, diag_res,
			static_cast<NDT>(num_data_id), out_fname, diag_grad_diff);
	} else if(data_id == diag_len - 1){// ssm
		if(diag_verbose){ printf("Generating SSMParam data\t"); }
		diag->generateSSMParamData(param_range, diag_res, out_fname);
	} else{
		printf("Data type: %d\n", data_id);
		throw mtf::utils::InvalidArgument("generateData:: Invalid data type provided");
	}
}

void generateInverseData(int data_id, int adt_len, int diag_len){
	char *out_fname = nullptr;
	if(data_id < adt_len){// analytical
		if(diag_verbose){
			printf("Generating inverse %s data\t", diag->toString(static_cast<ADT>(data_id)));
		}
		diag->generateInverseAnalyticalData(param_range, diag_res, static_cast<ADT>(data_id), out_fname);
	} else if(data_id < diag_len - 1){// numerical
		if(diag_verbose){
			printf("Generating numerical inverse %s data\t",
				diag->toString(static_cast<NDT>(data_id - adt_len)));
		}
		diag->generateInverseNumericalData(param_range, diag_res, static_cast<NDT>(data_id - adt_len), out_fname, diag_grad_diff);
	} else if(data_id == diag_len - 1){// ssm
		if(diag_verbose){ printf("Generating SSMParam data\t"); }
		diag->generateSSMParamData(param_range, diag_res, out_fname);
	} else{
		printf("Data type: %d\n", data_id);
		throw mtf::utils::InvalidArgument("generateInverseData:: Invalid data type provided");
	}
}

const char* getDataTypeName(int data_id, int adt_len, int diag_len){
	if(data_id < adt_len){// analytical
		return diag->toString(static_cast<ADT>(data_id));
	} else if(data_id < diag_len - 1){// numerical
		return diag->toString(static_cast<NDT>(data_id - adt_len));
	} else if(data_id == diag_len - 1){// ssm
		return "ssm_param";
	} else{
		throw mtf::utils::InvalidArgument("getDataTypeName:: Invalid datay type provided");
	}

}