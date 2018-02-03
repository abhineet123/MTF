This is a simple ROS interface for MTF called mtf_bridge. This provides two nodes - SharedImageWriter and sample_tracker_node. The former reads raw camera images and puts them in a buffer from where the latter can read them and use them for tracking.

In order to build this package, place the src/mtf_bridge folder inside the src folder of an existing catkin workspace and run catkin_make as usual.

The nodes can be run as:

rosrun mtf_bridge SharedImageWriter
rosrun mtf_bridge sample_tracker_node

SharedImageWriter subscribes to /camera/image_raw/ for input images so a camera capture node also needs to be running that publishes to this topic.



