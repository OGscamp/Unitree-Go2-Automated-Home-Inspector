#!/bin/bash

# Init conda into the shell
source ~/anaconda3/etc/profile.d/conda.sh

cd /home/m2m/isaac-go2-ros2
conda activate env_isaaclab

python isaac_go2_ros2.py