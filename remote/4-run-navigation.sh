#!/bin/bash

# Init conda into the shell
source ~/anaconda3/etc/profile.d/conda.sh

cd /home/m2m/
conda activate NavRL

ros2 launch navigation_runner navigation.launch.py