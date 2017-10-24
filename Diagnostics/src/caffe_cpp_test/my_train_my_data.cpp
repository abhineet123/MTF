#include <caffe/layers/memory_data_layer.hpp>
#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
#include <iostream>
#include "boost/scoped_ptr.hpp"
#include "caffe/util/db.hpp"
#include "caffe/util/io.hpp"

using namespace caffe;
using boost::scoped_ptr;
using namespace std;
using std::string;
using namespace cv;

std::vector<cv::Mat> train_data;
std::vector<int> train_labels;

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

/** Extract minibatch **/
void extract_batch(int bs, int bi, std::vector<cv::Mat> t_data, std::vector<int> t_labels, std::vector<cv::Mat> &t_data_b, std::vector<int> &t_labels_b)
{
    t_data_b.clear();
    t_labels_b.clear();
    for(int i=0 ; i<bs; i++)
    {
        t_data_b.push_back(t_data[bi*bs + i]);
        t_labels_b.push_back(t_labels[bi*bs + i]);
    }
}

/** Tests the Input Blobs and shows images and Label **/
void testing_input_blobs(shared_ptr<Net<float> > net_)
{
    cout<<"working"<<endl;
    std::vector<shared_ptr< Blob< float > > > all_blobs = net_->blobs();
    for (int i=0; i<all_blobs[0]->shape(0); i++)
    {
        int offset= all_blobs[1]->offset(i);
        cout<<"Label is "<<(int)all_blobs[1]->mutable_cpu_data()[offset]<<endl;
    
        offset= all_blobs[0]->offset(i);
        float* input_data = &all_blobs[0]->mutable_cpu_data()[offset];
        cv::Mat channel(all_blobs[0]->height(), all_blobs[0]->width(), CV_32FC1, input_data);
        cv::imshow("testing ", channel);
        cv::waitKey();
    }
//  for (int l=0; l<all_blobs.size(); l++)
//      cout<<"Input"<<l<<" "<<all_blobs[l]->shape_string()<<endl;

}

/** Training Function **/
int train(char *solver_file, char *weights_file, std::vector<cv::Mat> t_data, std::vector<int> t_labels ) {

    // Read Solver Parameters
    caffe::SolverParameter solver_param;
    caffe::ReadProtoFromTextFileOrDie(solver_file, &solver_param);
    
    //Get GPUs and set Mode of Caffe
    vector<int> gpus;
    get_gpus(&gpus);
    if (gpus.size() == 0) 
        Caffe::set_mode(Caffe::CPU);
    else
    {
        Caffe::set_mode(Caffe::GPU);
        caffe::Caffe::SetDevice(0);
    }

    //Create Solver Object
    shared_ptr<caffe::Solver<float> > solver(caffe::SolverRegistry<float>::CreateSolver(solver_param));
    
    //Load from Pretrained Model if exists
    if (strcmp(weights_file, ""))
    {
        cout<<"Loading pretrained weights from caffemodel"<<endl;
        solver->net()->CopyTrainedLayersFrom(weights_file);
    }
    
    cout<<"Starting training or finetuning"<<endl;

    int nepochs= 1000, disp_freq= 10, save_freq= 10, bs= 64;
    int nbatches= t_data.size()/bs;

    int iter=0;

    std::vector<cv::Mat> t_data_b;
    std::vector<int> t_labels_b;

    shared_ptr< caffe::Net<float> > net_=solver->net();
    for (int i=0; i<nepochs; i++)
    {
        for (int j=0; j<nbatches ; j++)
        {
            //extracts minibatch
            extract_batch(bs,j, t_data, t_labels, t_data_b, t_labels_b);
            
            //set Input Blob
            boost::dynamic_pointer_cast<MemoryDataLayer<float> >(net_->layers()[0])->AddMatVector(t_data_b, t_labels_b);

//            if(j>0)
//                testing_input_blobs(net_);

            //train the network for one iteration
            solver->Step(1);

            //Display loss
            if (iter%disp_freq ==0)
            {
                vector<Blob<float> *>out= net_->output_blobs();
                float loss= out[0]->asum_data();
                cout<<"Train Loss from iteration "<<iter<<" is "<<loss<<endl;
            }

            //save the network
            if (iter%save_freq==0)
            {
                NetParameter net_param;
                net_->ToProto(&net_param);
                WriteProtoToBinaryFile(net_param, "mnist.caffemodel");
                cout<<"Saving Params"<<endl;
            }
            iter++;
        }
    }

    NetParameter net_param;
    net_->ToProto(&net_param);
    WriteProtoToBinaryFile(net_param, "mnist.caffemodel");
    cout<<"Saving Params"<<endl;

    return 0;
}

/** Read from LMDB Files **/
void read_lmdb(char *path)
{
    scoped_ptr<db::DB> db(db::GetDB("lmdb"));
    db->Open(path, db::READ);
    scoped_ptr<db::Cursor> cursor(db->NewCursor());

    int count = 0;
    while (cursor->valid()) {
        Datum datum;
        datum.ParseFromString(cursor->value());
        DecodeDatumNative(&datum);

        cv::Mat img(datum.width(), datum.height(), CV_8UC1);
        img.data=(uchar *) datum.data().c_str();
        train_data.push_back(img.clone());

        train_labels.push_back(datum.label());
       
        ++count;
       cursor->Next();
    }
}

int main(int argc, char** argv) {

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0]
              << " solver.prototxt *network.caffemodel"<< std::endl;
    return 1;
  }

  char* solver_file   = argv[1];
  char *lmdb_file= "/home/lubicon/Code/caffe/examples/mnist/mnist_train_lmdb";
  read_lmdb(lmdb_file);
    cout<<"Read the LMDB"<<endl;
  char* trained_file= "";
  if (argc==3)
      string trained_file = argv[2];
  
  train(solver_file, trained_file, train_data, train_labels);

}


