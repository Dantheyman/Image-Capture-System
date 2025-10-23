#!/bin/bash


if [ "$EUID" -ne 0 ]; then
  SUDO='sudo'
else
  SUDO=''
fi


echo "Starting ROS Noetic and Catkin installation..."

#make sure curl in installed
apt update -y
apt install curl -y


# Setup keys
apt install -y curl gnupg2 lsb-release

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Setup sources.list
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list


#  Update package index
$SUDO apt update


#  Install ROS Noetic Desktop Full
$SUDO apt install -y ros-noetic-desktop-full

#  Install Catkin tools and dependencies
$SUDO apt install -y python3-catkin-tools python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep (required once)
$SUDO rosdep init || echo "rosdep already initialized"
rosdep update

# Environment setup - add to bashrc if not already added
if ! grep -Fxq "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    echo "Added ROS environment setup to ~/.bashrc"
fi
source /opt/ros/noetic/setup.bash

echo "ROS Noetic and Catkin installation completed!"

#build catkin workspace 
echo "Setting Up ICS"
cd ICS
catkin init
catkin build 

#dowload python dependencies 
echo "Downloading python dependencies"

sudo apt install -y python3-pip
sudo apt-get install -y python3-rospy
pip3 install roslibpy
pip3 install kivymd
pip3 install pyserial
pip3 install PyYAML
pip3 install sparkfun-ublox-gps
pip3 install spidev
sudo apt install python3-cv-bridge

python3 -m pip install /resources/pypylon-2.0.0rc1-cp38-cp38-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl


export PYTHONPATH=$PYTHONPATH:ICS/src

