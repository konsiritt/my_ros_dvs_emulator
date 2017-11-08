# my_ros_dvs_emulator
===========
Structure adapted from rpg_dvs_ros (https://github.com/uzh-rpg/rpg_dvs_ros)
Takes frames written to shared memory to process them to events and publish via ROS topic or to .aedat file (for jaer framework)

# Driver Installation

Tested only with Ubuntu 16.04 and ROS Kinetic

1. Install catkin tools:  
   `$ sudo apt-get install python-catkin-tools`

2. Create a catkin workspace (if you have not done it yet):  
   `$ cd`  
   `$ mkdir -p catkin_ws/src`  
   `$ cd catkin_ws`  
   `$ catkin config --init --mkdirs --extend /opt/ros/kinetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release`  

3. Clone the `catkin_simple` package (https://github.com/catkin/catkin_simple), which will be used to build the emulator packages:  
   `$ cd ~/catkin_ws/src`  
   `$ git clone https://github.com/catkin/catkin_simple.git`  

4. Clone this repository:  
   `$ cd ~/catkin_ws/src`  
   `$ git clone https://github.com/konsiritt/my_ros_dvs_emulator.git`  

5. Build the package:  
   `$ catkin build ros_dvs_emulator`  

6. Only a udev rule is needed to run the DVS driver. An installation script is provided in the package `libcaer_catkin`.  
  `$ roscd libcaer_catkin`  (need to source your setup.bash file first, or just do `$ cd libcaer_catkin`)  
  `$ sudo ./install.sh`

7. You can test the installation by running a provided launch file. It starts the driver:
   `$ roslaunch ros_dvs_emulator dvs_emulator.launch` 
the emulator only starts generating events once a TORCS process is running that fetches the frames via OpenGL. The corresponding TORCS fork can be found here: https://github.com/konsiritt/torcs-1.3.7

# Configure

Configuration of the emulator is done via setting parameters in the file ~/ros_dvs_emulator/include/ros_dvs_emulator/config_dvs.h
Settings include the resolution of the synthetic sensor (so far has to be consistent with the configuration specified in TORCS), the event threshold, output to ROS topics or .aedat file.


# Run

You can start the emulation by running a provided launch file. It starts the driver:
   `$ roslaunch ros_dvs_emulator dvs_emulator.launch` 
Additionally, another process with the TORCS simulation needs to be started. As soon as the implementation in TORCS (from https://github.com/konsiritt/torcs-1.3.7) writes frame data to shared memory, the emulator starts to generate synthetic events. 

