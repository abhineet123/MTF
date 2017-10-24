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

std::vector<cv::Mat> test_data;
std::vector<int> test_labels;

static bool PairCompare(const std::pair<float, int>& lhs,
                        const std::pair<float, int>& rhs) {
  return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<float>& v, int N) {
  std::vector<std::pair<float, int> > pairs;
  for (size_t i = 0; i < v.size(); ++i)
    pairs.push_back(std::make_pair(v[i], i));
  std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

  std::vector<int> result;
  for (int i = 0; i < N; ++i)
    result.push_back(pairs[i].second);
  return result;
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
        test_data.push_back(img.clone());

        test_labels.push_back(datum.label());
       
        ++count;
       cursor->Next();
    }
}

void predict(cv::Mat data, int label, boost::shared_ptr< caffe::Net<float> > net)
{
    //Set Input Blob
    std::vector<cv::Mat> v_data;
    v_data.push_back(data);
    std::vector<int> v_label;
    v_label.push_back(label);


    boost::dynamic_pointer_cast< caffe::MemoryDataLayer<float> >(net->layers()[0])->AddMatVector(v_data, v_label);

    //Forward data in the Network
    net->Forward();

    //Get output probabilities, and pick max class prob
    caffe::Blob<float> *out= net->output_blobs()[0];
    const float* begin = out->cpu_data();
    const float* end = begin + out->channels();
    std::vector<float> preds(begin, end);
    cout<<"The result is "<<preds[0]<<"While label is "<<label<<std::endl;

}


void test(char *model_file, char *params_file, std::vector<cv::Mat> t_data, std::vector<int> t_labels)
{
    //Load network
    boost::shared_ptr<caffe::Net<float> > net;
    net.reset(new caffe::Net<float> (model_file, TEST) );
    net->CopyTrainedLayersFrom(params_file);
    
    Caffe::set_mode(Caffe::GPU);
    caffe::Caffe::SetDevice(0);

    //Iterate on all data and send to test network
    for (int i=0; i<t_data.size(); i++)
        predict(t_data[i], t_labels[i], net);
}

int main(int argc, char** argv) {

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0]
              << " deploy.prototxt network.caffemodel"<< std::endl;
    return 1;
  }

  char* model_file   = argv[1];
  char *params_file = argv[2];

  char *lmdb_file= "/home/lubicon/Code/caffe/examples/mnist/mnist_test_lmdb";
  read_lmdb(lmdb_file);
    cout<<"Read the LMDB"<<endl;
  
  test(model_file, params_file, test_data, test_labels);

}


