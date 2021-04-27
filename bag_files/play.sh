#!/bin/bash
rosbag play -l $1 
rosrun image_transport republish compressed in:=/rs435d/color/image_raw raw out:=/camera/
