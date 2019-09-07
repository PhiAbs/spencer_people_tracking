# General Info

# Prerequisites
- Ubuntu 16.04 is recommended
- [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [ZED SDK](https://www.stereolabs.com/developers/release/) (see the [Installation Guide](https://www.stereolabs.com/docs/getting-started/installation/)) and its dependencies ([CUDA](https://developer.nvidia.com/cuda-downloads). Download the SDK version suitable for your Ubuntu version

# Dependencies
Assuming you have ROS Indigo or ROS Kinetic already installed, we recommend installing the required depencencies of our framework via:

    rosdep update
    rosdep install -r --from-paths . --ignore-src

# Installation
Create Catkin workspace and build the package

   sudo apt-get install python-catkin-tools
   mkdir -p catkin-ws/src
   cd catkin-ws
   catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   cd src
   git clone https://github.com/PhiAbs/spencer_people_tracking_yolo.git
   catkin build
   source devel/setup.bash

## Compile Darknet
We will use a fork of darknet from @AlexeyAB : https://github.com/AlexeyAB/darknet

- It is already present in the folder libdarknet

- Simply call make in the folder

        cd spencer_people_tracking_yolo/zed-yolo/libdarknet
        make -j4

- For more information regarding the compilation instructions, check the darknet Readme [here](../libdarknet/README.md)

