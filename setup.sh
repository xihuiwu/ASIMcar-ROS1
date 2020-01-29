#!/bin/bash

sudo apt install ros-melodic-gmapping
sudo apt install ros-melodic-robot-localization

git clone https://github.com/ros-perception/vision_opencv.git
cd sensor
git clone https://github.com/KristofRobot/razor_imu_9dof.git
