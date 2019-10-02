#!/bin/bash
rm -f /tmp/.X99-lock
rm -f /tmp/.X11-unix/X99 
./run_display.bash
source custom_ws/devel/setup.bash
roslaunch gymdt gymdt.launch
