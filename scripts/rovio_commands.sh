#!/bin/bash

#Author: Suyash Yeotikar
#This file contains helper functions to execute commands of the rovio package.
#Usage: source rovio_commands.sh . Then execute any of the functions below.

#function to clone rovio from github
function clone_rovio() {
  ROVIO_URL="git@github.com:suyash023/rovio.git"
  ROVIO_INTERFACES_URL="git@github.com:suyash023/rovio_interfaces.git"
  CURRENT_DIR=$(pwd)
  mkdir -p rovio_ws/src
  cd rovio_ws/src/
  git clone $ROVIO_URL
  git clone $ROVIO_INTERFACES_URL
  cd rovio
  git submodule update --init --recursive
  cd $CURRENT_DIR
}

#function to build rovio
function build_rovio() {
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
}

#function to install euroc datasets
#function install_euroc_datasets() {}

#function to install rovio dependencies
#function install_dependencies() {}

#function to install rovio scene depdencies
#function install_scene_dependencies() {}

#function to evaluate rovio on euroc datasets
function run_rovio_euroc() {
  EUROC_DATASETS_LOCATION=$(pwd)/datasets/machine_hall
  ROVIO_WS=$(pwd)/install/setup.bash
  dirList=($(ls ${EUROC_DATASETS_LOCATION}))
  source $ROVIO_WS
  for dir in "${dirList[@]}"; do

    ROS2_BAG_LOCATION=${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/${dir}_ros2.db3
    ROVIO_OUTPUT_LOCATION=${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/rovio
    echo "Deleting prevous rovio output location"
    rm -rf ${ROVIO_OUTPUT_LOCATION}
    echo "Processing dataset: ${ROS2_BAG_LOCATION}"
    if [ -f ${ROS2_BAG_LOCATION} ]; then
      ros2 launch rovio ros2_rovio_rosbag_node_launch.py rosbag_filename:=$ROS2_BAG_LOCATION
    else
      echo "Skipping dataset: ${dir}"
    fi
  done
}



