#include "mtf_bridge/SharedImageWriter.h"

SharedImageWriter::SharedImageWriter() : initialized(false), channels(3) {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Read in shared buffer name
    ros::NodeHandle nh_("~");
    nh_.param<std::string>("shm_name", shm_name, "SharedBuffer");
    ROS_INFO_STREAM("Writer read Param shm_name: " << shm_name);

    // Read in init_buffer topic
    nh.param<int>("buffer_count", buffer_count, 500);
    ROS_INFO_STREAM("Read Param buffer_count: " << buffer_count);

    // Read in init_buffer topic
    std::string image_topic;
    nh.param<std::string>("image_topic", image_topic, "/camera/image_raw");
    ROS_INFO_STREAM("Read Param image_topic: " << image_topic);
    image_sub = it.subscribe(image_topic, 1, &SharedImageWriter::new_image_cb, this);

    init_buffer_pub = nh.advertise<mtf_bridge::BufferInit>("init_buffer", 1000);
	image_index_pub = nh.advertise<std_msgs::UInt32>("input_image", 1000);
}

void SharedImageWriter::initialize_shared_buffer() {
    int frame_size = width * height * channels;
    shm = new boost::interprocess::shared_memory_object(boost::interprocess::open_or_create,
                                                        shm_name.c_str(),
                                                        boost::interprocess::read_write);
    // Sets the size of the shm.
    shm->truncate(frame_size * buffer_count);
    region = new boost::interprocess::mapped_region(*shm, boost::interprocess::read_write);
    initialized = true;

    init_msg.height = height;
    init_msg.width = width;
    init_msg.channels = channels;
    init_msg.buffer_count = buffer_count;
    init_msg.frame_size = frame_size;
    init_msg.shm_num = 0;
    init_msg.init_id = current_index;

    init_buffer_pub.publish(init_msg);
}

void SharedImageWriter::new_image_cb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        //TODO: Handle grey scale
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    int frame_size = width * height * channels;
    current_index = (current_index + 1) % buffer_count;
    void* address = static_cast<unsigned char*>(region->get_address()) + (current_index * frame_size * sizeof(unsigned char));
    memcpy(address, &cv_ptr->image.data[0], frame_size);

    if (!initialized) {
        width = cv_ptr->image.size().width;
        height = cv_ptr->image.size().height;
        initialize_shared_buffer();
    } else if (init_buffer_pub.getNumSubscribers() > 0){
        init_msg.init_id = current_index;
        init_buffer_pub.publish(init_msg);
    }

    // Publish index into the shared memory buffer of the new frame.
    std_msgs::UInt32 frame_msg;
    frame_msg.data= current_index;
    image_index_pub.publish(frame_msg);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "ImageWriter");
    SharedImageWriter image_writer;

	ros::Rate loop_rate(150);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}
