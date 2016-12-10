#ifndef SHAREDIMAGEWRITER_H
#define SHAREDIMAGEWRITER_H value
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "mtf_bridge/BufferInit.h"

// Shared memory buffer
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class SharedImageWriter {
public:
    SharedImageWriter ();

private:
    bool initialized;
    int height;
    int width;
    int channels;
    int buffer_count;
    int frame_size;
    int current_index;
    mtf_bridge::BufferInit init_msg;

    uchar** shared_mem_addrs;
    std::string shm_name;
    boost::interprocess::shared_memory_object* shm;
    boost::interprocess::mapped_region* region;
    cv::Mat **frame_buffer;

    ros::Publisher init_buffer_pub;
    ros::Publisher image_index_pub;
    image_transport::Subscriber image_sub;

    void initialize_shared_buffer();
    void new_image_cb(const sensor_msgs::ImageConstPtr& msg);
};
#endif /* ifndef SHAREDIMAGEWRITER_H */
