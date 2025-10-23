#!/bin/bash

source /opt/ros/noetic/setup.bash
source ICS/devel/setup.bash


CURDIR="$(pwd)"
# Append to PYTHONPATH
export PYTHONPATH="$PYTHONPATH:$CURDIR/ICS/src"

roslaunch  ICS/src/ICS/launch/ros_start.launch