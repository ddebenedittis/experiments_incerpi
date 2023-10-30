# Incerpi Experiments

## Overview

Experiments of Riccardo Incerpi of the control of a robot arm with soft actuators subject to temperature constraints.


## Preliminaries

If you want to use the Docker containerization:

Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine) with post-installation steps for Linux.

Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit) (nvidia-docker2).


## Usage

Build the docker image (use the `-r` option to update the underlying images) with
```shell
./build.bash [-r]
```

Run the Docker container with
```shell
./run.bash
```

Build the workspace with 
```shell
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash
```

Run the publishing node with
```shell
ros2 run commands_publisher commands_publisher_node
```

To visualize the results in RViz do
```shell
ros2 launch arm_description move_arm.launch.py
ros2 run commands_publisher commands_publisher_viz_node
```
