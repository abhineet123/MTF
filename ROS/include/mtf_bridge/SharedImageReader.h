#ifndef SHAREDIMAGEREADER_H
#define SHAREDIMAGEREADER_H value
#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include "mtf_bridge/BufferInit.h"

// Shared memory buffer
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class SharedImageReader {
public:
    SharedImageReader ();
    cv::Mat* get_next_frame();

    int get_height() { return height; };
    int get_width() { return width; };
    bool is_initialized() {return initialized;};

private:
    bool initialized;
    int height;
    int width;
    int channels;
    int buffer_count;
    int frame_size;
    int current_index;

    uchar** shared_mem_addrs;
    std::string shm_name;
    boost::interprocess::shared_memory_object* shm;
    boost::interprocess::mapped_region* region;
    cv::Mat **frame_buffer;

    ros::Subscriber init_buffer_sub;
    ros::Subscriber image_index_sub;

    void update_image_properties(mtf_bridge::BufferInitConstPtr buffer_init);
    void update_image_index(std_msgs::UInt32ConstPtr index);
    void initialize_shm();
};
#endif /* ifndef SHAREDIMAGEREADER_H */
