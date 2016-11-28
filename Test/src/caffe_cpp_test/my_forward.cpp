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
std::vector<cv::Mat> train_labels;

/** Tests the Input Blobs and shows images and Label **/
void testing_input_blobs(shared_ptr<Net<float> > net_)
{
    cout<<"working"<<endl;
    std::vector<shared_ptr< Blob< float > > > all_blobs = net_->blobs();
    for (int i=0; i<all_blobs[0]->shape(0); i++)
    {
    	int offset = all_blobs[1]->offset(i);
        std::cout<<"Label H*W*C "<<all_blobs[1]->height()<<" "<<all_blobs[1]->width()<<" "<<all_blobs[1]->channels()<<" ";

        float* input_label = &all_blobs[1]->mutable_cpu_data()[offset];
        for (int j = 0; j < all_blobs[1]->channels(); ++j) {
            std::cout<<" "<<input_label[j];
        }
        std::cout<<std::endl;

        offset= all_blobs[0]->offset(i);
        float* input_data = &all_blobs[0]->mutable_cpu_data()[offset];
        cv::Mat channel(all_blobs[0]->height(), all_blobs[0]->width(), CV_32FC1, input_data);
        cv::imshow("testing ", channel);
        cv::waitKey();
    }

}

void forward_pass(char *model_file, char *trained_file, std::vector<cv::Mat> t_data)
{

    // Load the network
    Caffe::set_mode(Caffe::GPU);
    caffe::Caffe::SetDevice(0);
   
    boost::shared_ptr<caffe::Net<float> > net_;

    net_.reset(new Net<float>(model_file, TEST));
    net_->CopyTrainedLayersFrom(trained_file);

    //Reshape bs to be 1 instead of batchsize
    std::vector<shared_ptr< Blob< float > > > all_blobs = net_->blobs();
    boost::shared_ptr<Blob<float> > input_layer = all_blobs[0];
    
    std::cout<<"Label H*W*C "<<all_blobs[0]->height()<<" "<<all_blobs[0]->width()<<" "<<all_blobs[0]->channels()<<std::endl;
    cout<<"Before change "<<all_blobs[0]->shape(0)<<std::endl;

    all_blobs[0]->Reshape(1, all_blobs[0]->channels(), all_blobs[0]->width(), all_blobs[0]->height() );
    all_blobs[1]->Reshape(1, all_blobs[1]->channels(), all_blobs[1]->width(), all_blobs[1]->height() );
    net_->Reshape();
    boost::dynamic_pointer_cast<MemoryDataLayer<float> >(net_->layers()[0])->set_batch_size(1);
    cout<<"After change "<<all_blobs[0]->shape(0)<<std::endl;

    // Iterate on data: set input blob + pass empty label
    cv::Mat dummy_label(1, 10, CV_32FC1);
    for(int j=0; j<t_data.size(); j++ )
    {
        cv::Mat d= t_data[j];
        std::vector<cv::Mat > vec_data;
        vec_data.push_back(d);
        
        std::vector<cv::Mat> vec_label;
        vec_label.push_back(dummy_label);

        cout<<"data type "<< vec_data[0].type()<<" "<<CV_32FC3<<endl;
        boost::dynamic_pointer_cast<MemoryDataLayer<float> >(net_->layers()[0])->AddMatVector(vec_data, vec_label);
        net_->Forward();

        // Take output from before last layer
        std::cout<<"Total number of blobs "<<all_blobs.size()<<std::endl;
        boost::shared_ptr<Blob<float> > output= all_blobs[all_blobs.size()-2];

        float* output_label = output->mutable_cpu_data();
        for (int j = 0; j < output->channels(); ++j) {
            std::cout<<" "<<output_label[j];
        }
        std::cout<<std::endl;

        testing_input_blobs(net_);

        std::cout<<"output blob "<<output->channels()<<" "<<output->width()<<" "<<output->height()<<std::endl;

    }
    
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

        int l= datum.label();
        float *label= new float[10];
        for (int j=0; j<10; j++)
        {
            if (j==l)
                label[j]=1;
            else
                label[j]=0;
        }
        cv::Mat mat_label(1, 10, CV_32FC1, label);
        train_labels.push_back(mat_label);

        ++count;
       cursor->Next();
    }
}


int main(int argc, char** argv) 
{

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0]
              << " model.prototxt network.caffemodel"<< std::endl;
    return 1;
  }

  char* model_file   = argv[1];
  char *lmdb_file= "/home/lubicon/Code/caffe/examples/mnist/mnist_train_lmdb";
  read_lmdb(lmdb_file);
  cout<<"Read the LMDB"<<endl;
  char *trained_file = argv[2];
 
  forward_pass(model_file, trained_file, train_data);

}


