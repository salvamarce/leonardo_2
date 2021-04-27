#!/bin/bash

rosbag record -o $1 /mavros/vision_pose/pose /rs435d/color/image_raw/compressed /rs435d/color/camera_info 