### Build the sample with catkin

Create a catkin workspace

        mkdir -p catkin-ws/src
        cd catkin-ws
        catkin config --init --cmake-args-DCMAKE_BUILD_TYPE=RelWithDebInfo

Clone and build package

        cd src
        git clone https://github.com/PhiAbs/zed-yolo.git
        cd ..
        catkin_make

## Compile Darknet

We will use a fork of darknet from @AlexeyAB : https://github.com/AlexeyAB/darknet

- It is already present in the folder libdarknet

- Simply call make in the folder

        cd libdarknet
        make -j4

- For more information regarding the compilation instructions, check the darknet Readme [here](../libdarknet/README.md)
