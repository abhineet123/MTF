This is a simple ROS interface for MTF called mtf_bridge. This provides two nodes - `SharedImageWriter` and `sample_tracker_node`.
The former reads raw camera images and puts them in a buffer from where the latter can read them and use them for tracking.

In order to build this package, create a soft link to or copy the _src/mtf_bridge_ folder into the _src_ folder of an existing catkin workspace. This can be done as:

`ln -s <MTF_DIR>/ROS/src/mtf_bridge <CATKIN_WS_DIR>/src/mtf_bridge`

or

`cp -r <MTF_DIR>/ROS/src/mtf_bridge <CATKIN_WS_DIR>/src/`

As mentioned in the "Building a new application that uses MTF" section of the main ReadMe, building MTF creates a file called `mtfConfig.cmake` in the build folder that is needed to configure projects correctly. This is also copied to the _src/mtf_bridge/cmake_ folder so creating a soft link is recommended so it updates automatically each time MTF is rebuilt. 
If the folder is copied instead, the copy command must be rerun each time MTF is rebuilt.

Finally, run `catkin_make` as usual to compile the nodes and run them as:

`rosrun mtf_bridge SharedImageWriter`

`rosrun mtf_bridge sample_tracker_node`

`SharedImageWriter` subscribes to `/camera/image_raw/` for input images so a camera capture node also needs to be running that publishes to this topic.

Also refer [this page](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) for more details on building and running ROS packages.



