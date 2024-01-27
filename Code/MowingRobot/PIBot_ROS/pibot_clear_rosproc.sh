#!/bin/bash
# kill all ros&pibot process
ps -aux | grep "/opt/ros" | grep -v "grep" | awk '{print $2}' | xargs kill -9
ps -aux | grep "/pibot_ros/ros_ws" | grep -v "grep" | awk '{print $2}' | xargs kill -9