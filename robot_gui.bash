#!/bin/bash
cd ~
cd Robot_GUI
source install/local_setup.bash
source /opt/ros/humble/local_setup.bash

/bin/bash -c 'ros2 run gui_launcher gui'

