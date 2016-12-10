#include "mtf_bridge/SharedImageReader.h"

SharedImageReader::SharedImageReader() : initialized(false) {
    ros::NodeHandle nh_("~");

    // Read in shared buffer name
    nh_.param<std::string>("shm_name", shm_name, "SharedBuffer");
    ROS_INFO_STREAM("Reader read Param shm_name: " << shm_name);

    // Read in init_buffer topic
    std::string init_topic;
    nh_.param<std::string>("init_topic", init_topic, "/init_buffer");
    ROS_INFO_STREAM("Read Param init_topic: " << init_topic);
    init_buffer_sub = nh_.subscribe(init_topic, 1, &SharedImageReader::update_image_properties, this);

    std::string image_index_topic;
    nh_.param<std::string>("image_index_topic", image_index_topic, "/input_image");
    ROS_INFO_STREAM("Read Param image_index_topic: " << image_index_topic);
    image_index_sub = nh_.subscribe(image_index_topic, 1, &SharedImageReader::update_image_index, this);
}

void SharedImageReader::update_image_index(std_msgs::UInt32ConstPtr index) {
    current_index = index->data;
}

void SharedImageReader::update_image_properties(mtf_bridge::BufferInitConstPtr buffer_init) {
    if (initialized)  {
        return;
    }
    initialized = true;
    ROS_INFO_STREAM("GOT IMAGE PROPERTIES");
    height = buffer_init->height;
    width = buffer_init->width;
    channels = buffer_init->channels;
    buffer_count = buffer_init->buffer_count;
    frame_size = buffer_init->frame_size;
    current_index = buffer_init->init_id;

    ROS_INFO_STREAM("Initialized image parameters");
    ROS_INFO_STREAM("height: " << height);
    ROS_INFO_STREAM("width: " << width);
    ROS_INFO_STREAM("channels: " << channels);
    ROS_INFO_STREAM("buffer_count: " << buffer_count);
    ROS_INFO_STREAM("frame_size: " << frame_size);
    ROS_INFO_STREAM("current_index: " << current_index);
    initialize_shm();
    init_buffer_sub.shutdown();
    ROS_INFO_STREAM("SHM initialized");
}

void SharedImageReader::initialize_shm() {
    // Opens read only shared region of memmory
    shm = new boost::interprocess::shared_memory_object(boost::interprocess::open_only,
            shm_name.c_str(),
            boost::interprocess::read_write);
    region = new boost::interprocess::mapped_region(*shm, boost::interprocess::read_only);
	uchar* start_addr = static_cast<uchar*>(region->get_address());

    // Create Mat objects for each of the frames in the buffer
	frame_buffer = new cv::Mat*[buffer_count];
    shared_mem_addrs = new uchar*[buffer_count];
    for(int i = 0; i < buffer_count; i++){
        frame_buffer[i] = new cv::Mat(height, width, CV_8UC3);
        shared_mem_addrs[i] = start_addr + (i * frame_size * sizeof(uchar));
        // Set the data pointer to be located in the correct place of the smh.
        frame_buffer[i]->data = shared_mem_addrs[i];
    }
}

cv::Mat* SharedImageReader::get_next_frame() {
    return frame_buffer[current_index];
}
