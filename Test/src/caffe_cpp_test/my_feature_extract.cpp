#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>

using namespace std;
using namespace caffe;  // NOLINT(build/namespaces)
using std::string;
using namespace cv;

cv::Mat convert_float_img(Mat &img)
{
    Mat imgU;
    double minVal; 
    double maxVal; 
    Point minLoc; 
    Point maxLoc;
    minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
    img -= minVal;
    img.convertTo(imgU,CV_8U,255.0/(maxVal-minVal));
    return imgU;
}


class FExtract {
 public:
  FExtract(const string& model_file,
           const string& trained_file,
           const string& mean_file);

  std::vector<cv::Mat> extract_features(cv::Mat img, std::string layer_name);
 
 private:
  void set_mean(const string& mean_file);
  void wrap_input_layer(std::vector<cv::Mat>* input_channels);
  void preprocess(const cv::Mat& img,std::vector<cv::Mat>* input_channels);
  void wrap_any_layer(std::vector<cv::Mat>* input_channels, shared_ptr< Blob<float> > layer);
  void wrap_any_params(std::vector<cv::Mat>* input_params, shared_ptr<Blob<float> > param_vector);

 private:
  shared_ptr<Net<float> > net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
};

FExtract::FExtract(const string& model_file,
                   const string& trained_file,
                   const string& mean_file) {
#ifdef CPU_ONLY
  Caffe::set_mode(Caffe::CPU);
#else
  Caffe::set_mode(Caffe::GPU);
#endif

  /* Load the network. */
  net_.reset(new Net<float>(model_file, TEST));
  net_->CopyTrainedLayersFrom(trained_file);

  CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

  Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  CHECK(num_channels_ == 3 || num_channels_ == 1)
    << "Input layer should have 1 or 3 channels.";
  input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

  /* Load the binaryproto mean file. */
  set_mean(mean_file);

}

/* Load the mean file in binaryproto format. */
void FExtract::set_mean(const string& mean_file) {
  BlobProto blob_proto;
  ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

  /* Convert from BlobProto to Blob<float> */
  Blob<float> mean_blob;
  mean_blob.FromProto(blob_proto);
  CHECK_EQ(mean_blob.channels(), num_channels_)
    << "Number of channels of mean file doesn't match input layer.";

  /* The format of the mean file is planar 32-bit float BGR or grayscale. */
  std::vector<cv::Mat> channels;
  float* data = mean_blob.mutable_cpu_data();
  for (int i = 0; i < num_channels_; ++i) {
    /* Extract an individual channel. */
    cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
    channels.push_back(channel);
    data += mean_blob.height() * mean_blob.width();
  }

  /* Merge the separate channels into a single image. */
  cv::Mat mean;
  cv::merge(channels, mean);

  /* Compute the global mean pixel value and create a mean image
   * filled with this value. */
  cv::Scalar channel_mean = cv::mean(mean);
  mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);
}

void FExtract::wrap_any_params(std::vector<cv::Mat>* input_params, shared_ptr<Blob<float> > param_vector) {
  int width = param_vector->width();
  int height = param_vector->height();

  cout<<"Wrap layer "<< width<<" "<<height<<endl;

  float* input_data =param_vector->mutable_cpu_data();
  for (int j=0; j<param_vector->shape(0); ++j)
  {
      for (int i = 0; i < param_vector->channels(); ++i) 
      {
        cv::Mat channel(height, width, CV_32FC3, input_data);
        input_params->push_back(channel);
        input_data += width * height*param_vector->channels();
      }
  }
}


void FExtract::wrap_any_layer(std::vector<cv::Mat>* input_channels, shared_ptr<Blob<float> > layer) {
  int width = layer->width();
  int height = layer->height();

  cout<<"Wrap layer "<< width<<" "<<height<<endl;

  float* input_data =layer->mutable_cpu_data();
  for (int i = 0; i < layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

std::vector<cv::Mat> FExtract::extract_features(cv::Mat img, std::string layer_name)
{
  Blob<float>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector<cv::Mat> input_channels;
  wrap_input_layer(&input_channels);

  preprocess(img, &input_channels);

  net_->ForwardPrefilled();

  const vector< shared_ptr< Blob< float > > > params= net_->params();
  cout<<"Vector size is "<<params.size()<<" "<<params[0]->shape(1)<<" "<<params[0]->shape(2)<<" "<<params[0]->shape(3)<<endl;

  const shared_ptr< Blob<float> > feature_blob= net_->blob_by_name(layer_name);
  cout<<"Extracted feature map from conv2 , size: ";
  //vector<int> v= feature_blob->shape();
  //for(int i=0; i<v.size(); i++)
    //cout<<v[i]<<" ";
  //cout<<endl;
  //cout<<"W H "<<feature_blob->width()<<" "<<feature_blob->height()<<endl;
  std::vector<Mat> fmap;
//  wrap_any_layer(&fmap,feature_blob); 
  wrap_any_layer(&fmap,params[0]); 
  cout<<"Size of Fmaps is "<<fmap.size()<<endl;
  for (int i=0; i<fmap.size(); i++)
  {
    cv::Mat mp= convert_float_img(fmap[i]);
    cout<<"Showing feature Map "<<i<<endl;
    imshow("Feature Map", mp);
    waitKey();
  }
  return fmap; 
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void FExtract::wrap_input_layer(std::vector<cv::Mat>* input_channels) {
  Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

void FExtract::preprocess(const cv::Mat& img,std::vector<cv::Mat>* input_channels) {
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, CV_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, CV_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, CV_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, CV_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0]
              << " deploy.prototxt network.caffemodel"
              << " mean.binaryproto labels.txt img.jpg" << std::endl;
    return 1;
  }

  ::google::InitGoogleLogging(argv[0]);

  string model_file   = argv[1];
  string trained_file = argv[2];
  string mean_file    = argv[3];
  FExtract fextract(model_file, trained_file, mean_file);

  string file = argv[4];

  cv::Mat img = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
  imshow("input image ", img);
  waitKey();
  CHECK(!img.empty()) << "Unable to decode image " << file;
  fextract.extract_features(img, "conv1");

}
