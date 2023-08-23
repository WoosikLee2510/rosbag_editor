#!/usr/bin/env bash

# available dataset
datadir="/home/wl/Desktop/husky"
dataset=(
"husky_indoor1"
"husky_indoor2"
"husky_outdoor1"
"husky_outdoor2"
"husky_outdoor3"
"husky_outdoor4"
"husky_outdoor5"
"husky_outdoor6"
"husky_outdoor7"
"husky_outdoor8"
"husky_trail1"
"husky_trail2"
"husky_trail3"
"husky_trail4"
)

for i in "${!dataset[@]}"; do
  roslaunch rosbag_editor rosbag_editor.launch bag:=${dataset[i]} source_path:="/media/wl/RPNG_FLASH_9" target_path:="/home/wl/Desktop/husky"
done