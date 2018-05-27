#!/bin/bash

# Set up ROS (lunar) on a fresh install, follwed by Gazebo (v8)
# intended non-GUI use for AWS


# Basic install steps for ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-lunar-ros-base
sudo rosdep init
rosdep update
echo "source /opt/ros/lunar/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# Add OSRF software sources for gazebo 8
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update

# install gazebo 8
sudo apt-get install -y ros-lunar-gazebo8-ros-pkgs
sudo apt-get install -y ros-lunar-gazebo8-ros-control # need control package for prius


# set up some other stuff
sudo apt-get install -y ros-lunar-robot-state-publisher
sudo apt-get install -y ros-lunar-tf-conversions ros-lunar-tf2-geometry-msgs
sudo apt-get install python-scipy python-matplotlib ipython # last one is just for convenience...

# install bigfloat and deps
sudo apt-get install libmpfr-dev
sudo apt-get install python-pip
pip install bigfloat

# set up catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
rm .catkin_workspace
rm -rf src

# clone in my repo
git init .
git remote add -t \* -f origin https://github.com/flyingsilverfin/AV-Camera-Placement.git
git pull origin master

# make messages etc
catkin_make

# on use will still need to run source ~/catkin_ws/devel/setup.bash
