# DeneigUS

DeneigUS is a robotics project created by seven engineering students at the University of Sherbrooke. The main objective of this projet is to design the **proof of concept of an autonomous and electric snowblower** (from mechanics to programming, including electrification).

The team implements several tools from [ROS Noetic](http://wiki.ros.org/noetic) on Ubuntu 20.04 for navigation and control. Also, the control is ensured by the fusion of several sensors, including GPS and IMU for localization, then sonars for the detection of obstacles while it is snowing.

## Folders Description

* **Altium:** This folder contains the PCB plans of the snowblower created with [Altium](https://www.altium.com/).
* **rospackage:** This folder contains all of the programming.


## Installation

### Cloning the repository 

1. Clone the package in your git directory on your computer:
```bash
git clone https://github.com/lefake/DeneigUS.git
```
### Installing ROS Noetic on Ubuntu 20.04
1. Follow the instructions on the offical website ([ROS Installation](http://wiki.ros.org/noetic/Installation/Ubuntu))
2. If you are not familiar with ROS, we strongly recommend that you do the tutorials ([ROS Tutorials](http://wiki.ros.org/ROS/Tutorials))

### Installing CoppeliaSim for use with ROS (Optional)
At the begining of the projet we used CoppeliaSim to simulate the snowblower and test our navigation with ROS. To install the CoppeliaSim simulator in order to use it with ROS, follow these steps:

1. Go to the official webstite of Coppelia Robotics and [download](https://www.coppeliarobotics.com/downloads) the free EDU version for Ubuntu 20.04.
2. Before configuring the simulator with ROS, you will need to install some libraries:
```bash
sudo apt-get install xsltproc
```
```bash
pip install xmlschema
```
3. In the User Manual of the simulator, do the ROS Tutorial in order to install the ROS Interface plugin ([Setting CoppeliaSim with ROS](https://www.coppeliarobotics.com/helpFiles/))


### Configuring your Catkin workspace
1. catkin_create_pkg deneigus:
2. clone this repo somewhere on your system
3. Make sym-link of all the files in the git folder to the ros folder 
    3.1 For each file except git files (.git/ and .gitignore) use this command "ln -s /ABS_GIT_PATH/file /ABS_ROS_PATH/
    3.2 For each folder you have to create the folder in the ROS directory first
4. Make all the src files (not needed for the .ino) in the GIT directory executable with chmod +x file
