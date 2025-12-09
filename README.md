# README #

## History and Present ##

This repo originally belonged to ETH Zurich under the MIT BSD Licence.
This fork of the original repo is now maintained by [robotboy_023](https://github.com/suyash023).
Maintaining the spirit from the original authors the licensing of this repo is also under MIT BSD.

As a Robotics Engineer I have worked extensively on ROVIO for years. I have gotten it to work on various resource constrained compute platforms and also made several modifications to improve its accuracy.
ROVIO is a relatively simple framework that can work quite well on low compute platforms on **all kinds of** Robots and provide accurate odometry **if tuned properly**.
I would like to leverage my years of experience to help out the community and improve ROVIO as a package offering.
I am happy to collaborate to improve ROVIO and you are more than welcome to suggest modifications, new features, improvement ideas or bug-fixes.
For this you can open an issue in the issues section of this repo.

If you would like to buy me some Chai (tea) to fuel the ongoing improvement work, please hit the sponsor button

I also offer troubleshooting and tuning services for your specific setup for a nominal fee. Please send an email or contact me through issues section to collaborate on this.


## Modifications and Roadmap ##

- [x] Support for ROS2
- [x] CI/CD pipeline for building 
  - [ ] ros2 parameters in yaml file
  - [ ] Publishing of odometry as transforms
  - [ ] Reset service calls
- [ ] Time-offset addition to IMU data.
- [ ] Update wiki
  - [ ] Sensor Calibration.
  - [ ] ROVIO parameter breakdown.
- [ ] Scripts
  - [ ] Install dependencies
  - [ ] Install EUROC dataset
  - [ ] Build ROVIO
  - [ ] Run ROVIO on all EUROC datasets
- [ ] Scripts to generate ATE and RPE metrics.
- [ ] Docker support and docker image.
- [ ] Optimizations from TU Delft.
- [ ] Optimizations from MAPLAB.
- [ ] Health monitoring cleanup.
- [ ] Lidar sensor fusion.
- [ ] Use stronger features first for update (sorting).
- [ ] Read calibration files directly from Kalibr and opencv.
- [ ] Sliding Window bundle Adjustment using GTSAM.
- [ ] Rolling Shutter Compensation.

## Modified README ##

Below readme contains modification from the original readme with additional information. Modifications have been made to the instructions for easier setup and to build and compile ROVIO
in accordance with the current working state.

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source (BSD License). Please remember that it is strongly coupled to on-going research and thus some parts are not fully mature yet. Furthermore, the code will also be subject to changes in the future which could include greater re-factoring of some parts.

Video: https://youtu.be/ZMAISVy-6ao

Papers:
* http://dx.doi.org/10.3929/ethz-a-010566547 (IROS 2015)
* http://dx.doi.org/10.1177/0278364917728574 (IJRR 2017)

Please also have a look at the wiki: https://github.com/ethz-asl/rovio/wiki


## Software Dependencies ##

### Install without openGL scene (Recommended) ##

* ros2 (Tested with desktop version of humble)
* kindr (https://github.com/ethz-asl/kindr) - A git submodule.
* ~~lightweight_filtering (as submodule, use "git submodule update --init --recursive")~~ : Lightweight_filtering has been removed as submodule from this repo. It is a static dependency now.
* rovio_interfaces : This is a package needed to use services and custom messages for ROVIO. ROS2 now mandates that messages and services be maintained in a separate package.

## Building ROVIO ##
Building has been tested with ROS2 humble, on Ubuntu 22.04.

Steps:

* Make a ros2 workspace in a desired (home here by default `~/`) folder:
```
#!command
mkdir -p ~/rovio_ws/src
```
* clone rovio and all the dependencies:
```
cd ~/rovio_ws/src/
git clone git@github.com:suyash023/rovio.git
git clone git@github.com:suyash023/rovio_interfaces.git
cd rovio
git submodule update --init -- recursive
```

* Build rovio
```
#!command
cd ~/rovio_ws/
colcon build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install with opengl scene ###
Addtional dependencies:
* opengl
* glut
* glew

Steps to build:
* Install additional dependencies:
```
sudo apt-get update
sudo apt-get install freeglut3-dev libglew-dev
```
* Follow the same instruction as above to create a ros2 workspace for rovio, on the build step execute the following command instead.
```
#!command

colcon build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

### ROVIO on Euroc datasets ###

### ROVIO on your robot ###

To run ROVIO on your custom camera-IMU setup please refer to this (documentation)[doc/CustomSetup.md]