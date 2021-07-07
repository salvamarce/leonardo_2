#!/bin/bash

rosbag record -o $1 /axis/image_raw/compressed /axis/state 
