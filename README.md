# DeneigUS

DeneigUS is a robotics project created by seven engineering students at the University of Sherbrooke. The main objective of this projet is to design the **proof of concept of an autonomous and electric snowblower** (from mechanics to programming, including electrification).

The team implements several tools from [ROS Noetic](http://wiki.ros.org/noetic) on Ubuntu 20.04 for navigation and control. Also, the control is ensured by a Rapberry Pi 4 with Arduinos and the fusion of several sensors, including GPS and IMU for localization, then sonars for the detection of obstacles while it is snowing. A user Interface is also implemented in HTML to visualize sensor data and error messages.

## Folders Description

* **Altium:** This folder contains the PCB plans of the snowblower created with [Altium](https://www.altium.com/).
* **rospackage:** This folder contains all of the programming.


## Installation

### ROS Noetic on Ubuntu 20.04
1. Follow the instructions on the offical website ([ROS Installation](http://wiki.ros.org/noetic/Installation/Ubuntu))
2. If you are not familiar with ROS, we strongly recommend that you do the tutorials ([ROS Tutorials](http://wiki.ros.org/ROS/Tutorials))

### The CoppeliaSim Simulator for use with ROS (Optional)
At the begining of the projet we used CoppeliaSim to simulate the snowblower and test our navigation with ROS. To install the CoppeliaSim simulator in order to use it with ROS, follow these steps:

1. Go to the official webstite of Coppelia Robotics and [download](https://www.coppeliarobotics.com/downloads) the free EDU version for Ubuntu 20.04.
2. Before configuring the simulator with ROS, you will need to install some libraries:
```bash
sudo apt-get install xsltproc
```
```bash
pip3 install xmlschema
```
3. In the user manual of the simulator, do the ROS Tutorial in Tutorials section, in order to install the ROS Interface plugin. ([Setting CoppeliaSim with ROS](https://www.coppeliarobotics.com/helpFiles/))
4. In your ~/.bashrc, add the following command and replace the path with the location of your CoppeliaSim folder on your system:
```bash
export COPPELIASIM_ROOT_DIR=put_your_path_here
```

### ROS Libraries for Navigation

The list of ROS libraries used in the project is as follows:
- [Joy](http://wiki.ros.org/joy) (To control the snowblower manually with a remote control)
- [Map_server](http://wiki.ros.org/map_server) (To generate a static map)
- [Costmap_2d](http://wiki.ros.org/costmap_2d) (To add obstacles layers to the static map)
- [Move_Base](http://wiki.ros.org/move_base) (To load and use Costmap_2d parameters)
- [Rosbrige Server](http://wiki.ros.org/rosbridge_suite) (In order to use our web Interface with ROS)

1. To install all ROS libraries needed in one command:
```bash
sudo apt-get install ros-noetic-joy ros-noetic-map-server ros-noetic-costmap-2d ros-noetic-move-base ros-noetic-rosbridge-server
```
2. Clone the [Range_Sensor_Layer](https://github.com/DLu/navigation_layers.git) in the src folder of your catkin workspace (we don't need social_navigation_layer and navigation_layers. We use only de Range Layer):
```bash
git clone https://github.com/DLu/navigation_layers.git
```

### Libraries for Communication Between Raspberry Pi and Arduino
1. Install pyserial:
```bash
pip3 install pyserial
```
2. Install Protobuf:
Clone or download the nanoPB library and move the folder into the Arduino's libraries folder (usually ~/snap/arduino/current/Arduino/libraries)
```bash
clone https://github.com/nanopb/nanopb.git
```


### Arduino Libraries for sensors
In Arduino IDE, go to Tools/Manage Libraries... and add the following libraries:
1.  MPU9250 by hideakitai (v.0.4.1). In the file MPU9250.h, add #include "Arduino.h" before #include <Wire.h>
2. SparkFun u-blox by SparkFun Electronics (last version) 

### Configuring your Catkin Workspace and Installation
1. Create a new folder named deneigus in your catkin/src
2. clone this repo somewhere on your system:
```bash
git clone https://github.com/lefake/DeneigUS.git
```
3. Make a sym-link of all the files in the git folder to the ros folder:
```bash
cp -rs /ABS_GIT_PATH/rospackage/* /ABS_ROS_PATH/catkin/src/deneigus/
```
4. Make the setup.sh file executable and run it:
```bash
chmod +x setup.sh;./setup.sh
```

## Usage

### Using the project in simulation with CoppeliaSim
1. Start ROS:
```bash
roscore
```
2. Start CoppeliaSim, open the simulation scene in the simulation folder and start the simulation.
3. Plug in your remote if you are using manual mode. 
4. Launch the deneigus launch file:
```bash
roslaunch deneigus deneigus.launch
```
### Using the projet on the real snowblower
1. Start ROS:
```bash
roscore
```
2. Plug in your remote if you are using manual mode. 
3. Launch the deneigus launch file:
```bash
roslaunch deneigus deneigus.launch
```
