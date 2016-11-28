#include "mtf/Utilities/netUtils.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/graphUtils.h"

_MTF_BEGIN_NAMESPACE

namespace utils
{

	/****
	 *Implementation of Global Functions
	 * ****/

MTFNet* createNetwork(RegNetParams rgparams, char *conv_model, char *last_conv_layer,
	int n_layers, int *n_neurons, char *activ_fn){
		//Creation of Conv layers only, to add fully connected layers
        std::cout<<"Received "<<rgparams.nepochs<<" "<<rgparams.bs<<" "<<rgparams.debug<<" "<<rgparams.enable_preproc<<" "<<rgparams.solver_file
            <<" "<<rgparams.train_file<<" "<<rgparams.mean_file<<" "<<rgparams.load_pretrained<<std::endl;

        MTFNet *net= new MTFNet(rgparams);
		return net;
	}


	void testingInputBlobs(boost::shared_ptr<caffe::Net<float> > net_){
		std::vector<boost::shared_ptr< caffe::Blob< float > > > all_blobs = net_->blobs();
		for(int i = 0; i < all_blobs[0]->shape(0); i++)
		{
			int offset = all_blobs[1]->offset(i);
			std::cout << "Label H*W*C " << all_blobs[1]->height() << " " << all_blobs[1]->width() << " " << all_blobs[1]->channels() << " ";

			float* input_label = &all_blobs[1]->mutable_cpu_data()[offset];
			for(int j = 0; j < all_blobs[1]->channels(); ++j) {
				std::cout << " " << input_label[j];
			}
			std::cout << std::endl;

			offset = all_blobs[0]->offset(i);
			float* input_data = &all_blobs[0]->mutable_cpu_data()[offset];
			cv::Mat img(all_blobs[0]->height(), all_blobs[0]->width(), CV_32FC3);
			std::vector<cv::Mat> in_channels;
			int height = all_blobs[0]->height();
			int width = all_blobs[0]->width();
			for(int i = 0; i < all_blobs[0]->channels(); ++i) {
				cv::Mat channel(height, width, CV_32FC1, input_data);
				in_channels.push_back(channel);
				input_data += width * height;
			}
			cv::merge(in_channels, img);
			cv::Mat imgU = utils::convertFloatImgToUchar(img, 3);
			cv::imshow("testing ", imgU);
			cv::waitKey();
        
            offset= all_blobs[all_blobs.size()-2]->offset(i);
            float* output_label = &all_blobs[all_blobs.size()-2]->mutable_cpu_data()[offset];
            std::cout<<"Output of Regression "<<std::endl;
            for (int j = 0; j < all_blobs[all_blobs.size()-2]->channels(); ++j) 
                std::cout<<" "<<output_label[j];
            std::cout<<std::endl;
		}

    }

float *forwardPass(MTFNet *network, cv::Mat patch)
    {
        boost::shared_ptr<caffe::Net<float> > net_= network->solver_->net();

        //Reshape bs to be 1 instead of batchsize
        std::vector<boost::shared_ptr<caffe::Blob< float > > > all_blobs = net_->blobs();
        boost::shared_ptr<caffe::Blob<float> > input_layer = all_blobs[0];
        
        std::cout<<"Label H*W*C "<<all_blobs[0]->height()<<" "<<all_blobs[0]->width()<<" "<<all_blobs[0]->channels()<<std::endl;
        cout<<"Before change "<<all_blobs[0]->shape(0)<<std::endl;

        all_blobs[0]->Reshape(1, all_blobs[0]->channels(), all_blobs[0]->width(), all_blobs[0]->height() );
        all_blobs[1]->Reshape(1, all_blobs[1]->channels(), all_blobs[1]->width(), all_blobs[1]->height() );
        net_->Reshape();
        boost::dynamic_pointer_cast<MemoryDataLayer<float> >(net_->layers()[0])->set_batch_size(1);
        cout<<"After change "<<all_blobs[0]->shape(0)<<std::endl;

        // Iterate on data: set input blob + pass empty label
        cv::Mat dummy_label(1, 10, CV_32FC1);
        std::vector<cv::Mat > vec_data;
        vec_data.push_back(patch);
        if (network->rgparams.enable_preproc)
            network->preprocess_batch(vec_data);

        std::vector<cv::Mat> vec_label;
        vec_label.push_back(dummy_label);

        boost::dynamic_pointer_cast<MemoryDataLayer<float> >(net_->layers()[0])->AddMatVector(vec_data, vec_label);
        net_->Forward();
    
        if (network->rgparams.debug)
            testing_input_blobs(net_);

        // Take output from before last layer
        std::cout<<"Total number of blobs "<<all_blobs.size()<<std::endl;
        boost::shared_ptr<Blob<float> > output= all_blobs[all_blobs.size()-2];

        float* output_label = output->mutable_cpu_data();
        for (int j = 0; j < output->channels(); ++j) {

            std::cout<<" "<<output_label[j];
        }
        std::cout<<std::endl;
        return output_label;
    }

	void train(MTFNet *network, std::vector<cv::Mat> training_data, std::vector<cv::Mat> training_labels){
		char *save_file = "temp.caffemodel";
		cout << "Starting training or finetuning" << endl;

		int disp_freq = 10;
		int nbatches = training_data.size() / network->rgparams.bs;

		int iter = 0;

		std::vector<cv::Mat> t_data_b;
		std::vector<cv::Mat> t_labels_b;
        
        VectorXf losses(network->rgparams.nepochs*nbatches);

        std::cout<<"nepochs is "<<network->rgparams.nepochs<<" "<<network->rgparams.bs<<" "<<network->rgparams.enable_preproc<<std::endl;
		for(int i = 0; i < network->rgparams.nepochs; i++)
		{
            float loss_perepoch=0;
			for(int j = 0; j < nbatches; j++)
			{
				//extracts minibatch
				network->extract_batch(network->rgparams.bs, j, training_data, training_labels, t_data_b, t_labels_b);
                if (network->rgparams.enable_preproc)
                    network->preprocess_batch(t_data_b);

				//set Input Blob
				boost::dynamic_pointer_cast<MemoryDataLayer<float>>(network->net_->layers()[0])->AddMatVector(t_data_b, t_labels_b);

				//train the network for one iteration
				network->solver_->Step(1);

                if(network->rgparams.debug)
                    testing_input_blobs(network->net_);

        		vector<caffe::Blob<float> *>out = network->net_->output_blobs();
				losses(iter) = out[0]->asum_data();
				loss_perepoch+= losses(iter);

				//Display loss
				if(iter%disp_freq == 0)
				{
				    cout << "Train Loss from iteration " << iter << " is " << losses(iter) << endl;
				}
				iter++;

			}
            std::cout<<"Train loss per epoch"<<loss_perepoch/(float)nbatches<<std::endl;
		}

        cv::imshow("testing", cv::Mat(utils::drawFloatGraph(losses.data(), losses.size()) ) );
        cv::waitKey(1);

        network->net_= network->solver_->net();
		caffe::NetParameter net_param;
		network->net_->ToProto(&net_param);
		caffe::WriteProtoToBinaryFile(net_param, save_file);
		cout << "Saving Params" << endl;

	}

	/****
	 *Implementation of MTFNet functionalities
	 * ****/
	MTFNet::MTFNet(RegNetParams _rgparams)
	{
       rgparams= _rgparams; 

		// Read Solver Parameters
		caffe::SolverParameter solver_param;
		caffe::ReadProtoFromTextFileOrDie(_rgparams.solver_file, &solver_param);

		//Get GPUs and set Mode of Caffe
		vector<int> gpus;
		getGPUs(&gpus);
		if(gpus.size() == 0)
			Caffe::set_mode(Caffe::CPU);
		else
		{
			Caffe::set_mode(Caffe::GPU);
			caffe::Caffe::SetDevice(0);
		}

		//Create Solver Object
		solver_ = boost::shared_ptr< caffe::Solver<float> >(caffe::SolverRegistry<float>::CreateSolver(solver_param));
		net_ = solver_->net();

		//Load from Pretrained Model if exists
		if(_rgparams.train_file != NULL)
		{
			cout << "Loading pretrained weights from caffemodel" << endl;
			solver_->net()->CopyTrainedLayersFrom(_rgparams.train_file);
		}

        //Set Mean
        if(_rgparams.enable_preproc)
        {
            std::cout<<"Setting the mean"<<std::endl;
            BlobProto blob_proto;
            ReadProtoFromBinaryFileOrDie(_rgparams.mean_file, &blob_proto);
       
            /* Convert from BlobProto to Blob<float> */
            Blob<float> mean_blob;
            mean_blob.FromProto(blob_proto);

            /* The format of the mean file is planar 32-bit float BGR or grayscale. */
            std::vector<cv::Mat> channels;
            float* data = mean_blob.mutable_cpu_data();
            for (int i = 0; i < 3; ++i) {
                /* Extract an individual channel. */
                cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
                channels.push_back(channel);
                data += mean_blob.height() * mean_blob.width();
            }
            std::cout<<"computing channel mean"<<std::endl;

            /* Merge the separate channels into a single image. */
            cv::Mat mean;
            cv::merge(channels, mean);

            /* Compute the global mean pixel value and create a mean image
            * filled with this value. */
            channel_mean_ = cv::mean(mean);
        }
	}

	void MTFNet::getGPUs(vector<int>* gpus)
	{
		int count = 0;

#ifndef CPU_ONLY
		CUDA_CHECK(cudaGetDeviceCount(&count));
#else
		NO_GPU;
#endif

		for(int i = 0; i < count; ++i)
			gpus->push_back(i);
	}

	void MTFNet::preprocessBatch(std::vector<cv::Mat>& data){
		cv::Mat mean = cv::Mat(data[0].size(), data[0].type(), channel_mean_);
		for(int i = 0; i < data.size(); i++)
			cv::subtract(data[i], mean, data[i]);
	}

	void MTFNet::extractBatch(int bs, int bi, std::vector<cv::Mat> t_data, 
		std::vector<cv::Mat> t_labels, std::vector<cv::Mat> &t_data_b, 
		std::vector<cv::Mat> &t_labels_b){
		t_data_b.clear();
		t_labels_b.clear();
		for(int i = 0; i < bs; i++)
		{
			t_data_b.push_back(t_data[bi*bs + i]);
			t_labels_b.push_back(t_labels[bi*bs + i]);
		}
	}

}
_MTF_END_NAMESPACE
