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

#fucntion to build rovio with scene
function build_rovio_scene() {
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
}

#function to convert euroc datasets from ros 1 to ros2 bag formats
function convert_euroc_rosbags(){
  DATASETS_DIR=$(pwd)/datasets/machine_hall
  folders=($( ls $DATASETS_DIR))
  for fol in "${folders[@]}"; do
    ROS2_DIR=$DATASETS_DIR/$fol/${fol}_ros2
    if [[ -d $ROS2_DIR ]]; then
      echo "ROS2 dir exists. No need to convert dataset: ${ROS2_DIR}"
    else
      echo "Converting dataset: ${fol}"
      rosbags-convert --dst ${ROS2_DIR} $DATASETS_DIR/${fol}/${fol}.bag
    fi

  done

}

#function to build rovio in debug mode
function build_rovio_debug() {
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
}

#function to install euroc datasets
#function install_euroc_datasets() {}


#fuction to install evaluation dependecies
#Note evo installation might require specific version dependencies.
#Follow the debug messages and install the specific versions
function install_evaluation_dependencies() {
  pip install evo
}

#function to install dataset conversion dependencies
function install_dataset_dependencies() {
  sudo apt-get install python3-pip
  pip install rosbags
}


#function to check if ROS2 is installed
function check_ros2_install() {
  ROS_LOCATION="/opt/ros"
  if [[ -d $ROS_LOCATION ]]; then
    echo "yes"
  else
    echo "no"
  fi
  return 0
}

#function to install ROS2 if not alredy present
function install_ros2() {
  ROS_DISTRO="humble"
  install=check_ros2_install
  if [[ $install == "yes" ]]; then
    echo "ROS2 found! Skipping ROS2 install"
  else
    echo "Installing ROS2: $ROS_DISTRO"
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-${ROS_DISTRO}-desktop
    sudo apt install ros-dev-tools
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
  fi
}


#function to install rovio dependencies
function install_dependencies() {
  install_ros2
}

#function to install rovio scene depdencies
function install_scene_dependencies() {
  sudo apt-add-repository universe
  sudo apt-get update
  sudo apt-get install freeglut3-dev libglew-dev
}

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
      echo "Skipping dataset: ${dir}, no ros2 bag file found"
    fi
  done
}



