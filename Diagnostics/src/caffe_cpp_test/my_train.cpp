#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
#include <iostream>

#include <gflags/gflags.h>
//#include <glog/logging.h>

using namespace std;
using namespace caffe;
using std::string;
using namespace cv;
/*using caffe::Blob;
using caffe::Caffe;
using caffe::Net;
using caffe::Layer;
using caffe::Solver;
using caffe::shared_ptr;
using caffe::string;
using caffe::vector;
*/
// Parse GPU ids or use all available devices
static void get_gpus(vector<int>* gpus) {
    int count = 0;

#ifndef CPU_ONLY
    CUDA_CHECK(cudaGetDeviceCount(&count));
#else
    NO_GPU;
#endif

    for (int i = 0; i < count; ++i)
      gpus->push_back(i);
}

int train(char *solver_file, char *weights_file) {
  caffe::SolverParameter solver_param;
  caffe::ReadProtoFromTextFileOrDie(solver_file, &solver_param);

  vector<int> gpus;
  get_gpus(&gpus);
  if (gpus.size() == 0) {
    LOG(INFO) << "Use CPU.";
    Caffe::set_mode(Caffe::CPU);
  } else {
      cout<<"Using GPU "<<gpus.size()<<endl;
    ostringstream s;
    for (int i = 0; i < gpus.size(); ++i) {
      s << (i ? ", " : "") << gpus[i];
    }
    LOG(INFO) << "Using GPUs " << s.str();
#ifndef CPU_ONLY
    cudaDeviceProp device_prop;
    for (int i = 0; i < gpus.size(); ++i) {
      cudaGetDeviceProperties(&device_prop, gpus[i]);
      LOG(INFO) << "GPU " << gpus[i] << ": " << device_prop.name;
    }
#endif
    solver_param.set_device_id(gpus[0]);
    Caffe::SetDevice(gpus[0]);
    Caffe::set_mode(Caffe::GPU);
    Caffe::set_solver_count(gpus.size());
  }

  shared_ptr<caffe::Solver<float> > solver(caffe::SolverRegistry<float>::CreateSolver(solver_param));

  if (strcmp(weights_file, ""))
  {
      cout<<"Loading pretrained weights from caffemodel"<<endl;
      solver->net()->CopyTrainedLayersFrom(weights_file);
  }
    
/*  cout<<"Network layers "<<endl;
  vector<string> names= solver->net()->layer_names();
  for (int i=0; i<names.size() ; i++)
  {
      cout<<names[i]<<endl;
  }*/
  cout<<"Starting training or finetuning"<<endl;
//  solver->Step(100);
  solver->Solve();
  
  return 0;
}

int main(int argc, char** argv) {

  FLAGS_alsologtostderr = 1;
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0]
              << " solver.prototxt *network.caffemodel"<< std::endl;
    return 1;
  }

  //google::InitGoogleLogging(argv[0]);

  char* solver_file   = argv[1];
  char* trained_file= "";
  if (argc==3)
      string trained_file = argv[2];
  train(solver_file, trained_file);

}


