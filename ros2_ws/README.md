# ROS2 Set Up

This is a guide for the ROS2 set up used for recording data from various sensors - camera, radar and the steering system.

## Setup ROS2 on RubikPi 
The RubikPi has Ubuntu 24.04 installed on it. We install ROS2 Jazzy, the version of ROS2 compatible with Ubuntu 24.04.

Refer to the <a href="https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html">ROS2 Jazzy documentation</a> for installation details. A brief summary of the same is also included in this README.

### Set Locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Enable the Ubuntu Universe Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Install Development Tools
```bash
sudo apt update && sudo apt install ros-dev-tools
```

### Install ROS2
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```

### Testing the installation
In one terminal source ROS2 and run the talker node.
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_python talker
```
In another terminal, source ROS2 again and run the listener node.
```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

You should see the listener displaying the messages published by the talker. ROS2 has been successfully setup on the machine.

## Running ROS2 Nodes
### Clone the repository 
Clone the repository and navigate to the ROS2 folder
```bash
git clone https://github.com/tomaszfrelek2/237D.git
cd ros2_ws
```

### Build the Package
Run `colcon build` to build all the packages. A very nice article for the rationale behind for a unified package builder can be found at: <a href="https://design.ros2.org/articles/build_tool.html">https://design.ros2.org/articles/build_tool.html</a>

### Running ROS2 Nodes
Source the setup files with `source install/setup.bash` 
Start the camera node using 
```bash
ros2 run scooter_data camera
```

Start the radar node using 
```bash
ros2 run scooter_data radar
```

Start the steering node using 
```bash
ros2 run scooter_data servo
```

Start the YOLO node using 
```bash
ros2 run scooter_data camera_yolov26_node
```

Alternatively, you can start all the nodes together using the launch file
```bash
ros2 launch scooter_data scooterdata_launch.py
```

Use `ros2 topic list` to see the `/topics` that are being published the nodes. Run `ros2 run scooter_data bag_recorder` to record the data using ROS2 Bag. ROS2 Bag is a tool for recording and playback of topics published by a ROS2 Node.

To visualise the saved data, run `python3 playback.py --"/path_to_bag" --o "/save_directory"