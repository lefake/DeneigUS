# DeneigUS

DeneigUS is a robotics project created by seven engineering students at the University of Sherbrooke. The main objective of this projet is to design the **proof of concept of an autonomous and electric snowblower** (from mechanics to programming, including electrification).

The team implements several tools from [ROS Noetic](http://wiki.ros.org/noetic) on Ubuntu 20.04 for navigation and control. Also, the control is ensured by a Rapberry Pi 4 with Arduinos and the fusion of several sensors, including GPS and IMU for localization, then sonars for the detection of obstacles while it is snowing.

## Folders Description

* **Altium:** This folder contains the PCB plans of the snowblower created with [Altium](https://www.altium.com/).
* **rospackage:** This folder contains all of the programming.


## Installation

### Installation of ROS Noetic on Ubuntu 20.04
1. Follow the instructions on the offical website ([ROS Installation](http://wiki.ros.org/noetic/Installation/Ubuntu))
2. If you are not familiar with ROS, we strongly recommend that you do the tutorials ([ROS Tutorials](http://wiki.ros.org/ROS/Tutorials))

### Installion of CoppeliaSim for use with ROS (Optional)
At the begining of the projet we used CoppeliaSim to simulate the snowblower and test our navigation with ROS. To install the CoppeliaSim simulator in order to use it with ROS, follow these steps:

1. Go to the official webstite of Coppelia Robotics and [download](https://www.coppeliarobotics.com/downloads) the free EDU version for Ubuntu 20.04.
2. Before configuring the simulator with ROS, you will need to install some libraries:
```bash
sudo apt-get install xsltproc
```
```bash
pip install xmlschema
```
3. In the user manual of the simulator, do the ROS Tutorial in order to install the ROS Interface plugin ([Setting CoppeliaSim with ROS](https://www.coppeliarobotics.com/helpFiles/))

### Installation of ROS Libraries for Navigation

1. Install [Joy](http://wiki.ros.org/joy) (To control the snowblower manually with a remote control):
```bash

```

2. Install [Map_server](http://wiki.ros.org/map_server) (To generate a static map):
```bash

```
3. Install [Costmap_2d](http://wiki.ros.org/costmap_2d) (To add obstacles layers to the static map):
```bash

```
4. Install [Move_Base](http://wiki.ros.org/move_base) (To load and use Costmap_2d parameters):
```bash

```
5. Clone the Range_Sensor_Layer in the src folder of your catkin workspace:
```bash

```
6. Install the [Rosbrige Server](http://wiki.ros.org/rosbridge_suite) (In order to use our web Interface with ROS):
```bash

```

### Installations for Communication Between Raspberry Pi and Arduino
1. Install pyserial:
```bash

```
2. Install Protobuf:
```bash

```

### Installation of Arduino Libraries for sensors
1. Install MPU9250 -> hideakitai (v.0.4.1):
2. In the header file of this library, add this instruction: #include "Arduino.h"
3. Install SparkFun U-Blox (last version):
4. 

### Configuring your Catkin Workspace and Installation
1. Create a new ROS package named deneigus in your catkin workspace
2. clone this repo somewhere on your system:
```bash
git clone https://github.com/lefake/DeneigUS.git
```
3. Make a sym-link of all the files in the git folder to the ros folder:
```bash
ln -s /ABS_GIT_PATH/file /ABS_ROS_PATH/
```
4. Rendre les fichiers python exécutables??

5. Make all the src files (not needed for the .ino) in the GIT directory executable with chmod +x file
