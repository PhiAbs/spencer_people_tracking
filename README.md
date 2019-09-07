# General Info

# Prerequisites
- Cuda 10.0
- Nvidia Driver: 430.26
- Ubuntu 18.04
- ZED SDK for Ubuntu 18.04 and Cuda 10
- ROS Melodic
- cuDNN (but I did not install that yet)
- opencv: sudo apt install pkg-config libopencv-dev

# Dependencies
Assuming you have ROS Indigo or ROS Kinetic already installed, we recommend installing the required depencencies of our framework via:

    rosdep update
    rosdep install -r --from-paths . --ignore-src

# Installation
Create Catkin workspace and build the package:

    sudo apt-get install python-catkin-tools
    mkdir -p catkin-ws/src
    cd catkin-ws
    catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cd src
    git clone https://github.com/PhiAbs/spencer_people_tracking_yolo.git

Make sure that you compile with c++11! If not, add the following line to CMakeLists.txt

    add_compile_options(-std=c++11)

Then

    catkin build -c -s
    source devel/setup.bash

## Compile Darknet
We will use a fork of darknet from @AlexeyAB : https://github.com/AlexeyAB/darknet

- It is already present in the folder libdarknet

- Simply call make in the folder

        cd spencer_people_tracking_yolo/zed-yolo/libdarknet
        make -j4

- For more information regarding the compilation instructions, check the darknet Readme [here](../libdarknet/README.md)

