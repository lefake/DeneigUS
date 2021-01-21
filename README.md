# DeneigUS
Projet de fin de bac UdeS

In order to clone this repo and use it with ROS (as a ROS package) you will have to do the following : 

1. catkin_create_pkg deneigus
2. clone this repo somewhere on your system
3. Make sym-link of all the files in the git folder to the ros folder 

    3.1 For each file except git files (.git/ and .gitignore) use this command "ln -s /ABS_GIT_PATH/file /ABS_ROS_PATH/
    
    3.2 For each folder you have to create the folder in the ROS directory first
4. Make all the src files (not needed for the .ino) in the GIT directory executable with chmod +x file
