# ABP-Simulation-Platform

This repository is dedicated to the development and simulation of floatbots (or satlets) on the air-bearing platform of the Space Engineering Research Center (SERC). SERC is a subsidiary of the Information Sciences Institute (ISI) of the University of Southern California (USC).

![Floatbots](https://github.com/koverman47/ABP-Simulation-Platform/blob/main/images/intro.jpg?raw=true)

## Getting Started
#### System Recommendations
* [Ubuntu 18.04.5 - Bionic Beaver](https://releases.ubuntu.com/18.04/)
* [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
* [Gazebo 9.15](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0)
  * Follow Alternative Instructions to get 9.15 instead of 9.0
  
#### Building and Configuring Path
Create a fork of this repository and then clone your fork to your local machine. To ensure ROS is in your environment run:
```bash
echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
Next let's build this repository as an ROS workspace. If you cloned the repository into your home directory:
```bash
cp ~/ABP-Simulation-Platform
catkin_make
source devel/setup.bash
```
Now to add the repository's packages permanently to your path:
```bash
echo 'source $HOME/Desktop/ABP-Simulation-Platform/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
Lastly, to ensure the robot models are in your path:
```bash 
echo 'export GAZEBO_MODEL_PATH=/home/kirby/Desktop/ABP-Simulation-Platform/src/abp_sim/models' >> ~/.bashrc
source ~/.bashrc
```

## Running a Toy Simulation
```bash
cd src/abp_sim/
./abp 3 --build
```
If everything is installed properly, you should initially see three floatboats at rest as seen in the above image.

## Troubleshooting
If you run into issues with Gazebo or ROS, see the following links as starting points:
* [Roslaunch with Gazebo](http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros)
* [Model is Invisible in Gazebo](https://answers.gazebosim.org//question/19654/my-model-is-not-showing-in-gazebo/)
* [Spawning Multiple Robots in Gazebo with ROS](https://answers.ros.org/question/315010/spawn-three-different-custom-robots-in-a-launch-file/)
