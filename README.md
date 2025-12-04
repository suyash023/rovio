# README #

## History and Present ##

This repo originally belonged to ETH Zurich under the MIT BSD Licence.
This fork of the original repo is now maintained by [robotboy_023](https://github.com/suyash023).
Maintaining the spirit from the original authors the licensing of this repo is also under MIT BSD.

As a Robotics Engineer I have worked extensively on ROVIO for years. I have gotten it to work on various resource constrained compute platforms and also made several modifications to improve its accuracy.
ROVIO is a relatively simple framework that can work quite well on low compute platforms on Robots and provide good accuracy **if tuned properly**.
I would like to leverage my years of experience to help out the community and improve ROVIO as a package offering.
I am happy to collaborate to improve ROVIO and you are more than welcome to suggest modifications, new features, improvement ideas or bug-fixes.
For this you can open an issue in the issues section of this repo.

If you would like to buy me some Chai (tea) to fuel the ongoing improvement work, please visit this [link](https://www.patreon.com/c/robotboy023/posts)

## Modifications and Roadmap ##

- [x] Support for ROS2
  - [ ] ros2 parameters in yaml file
  - [ ] Publishing of odometry as transforms
  - [ ] Reset service calls 
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

## README from Original Authors ##

Below is the README from original authors. Modifications have been made to the instruction to build and compile ROVIO
in accordance with the current working state.

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source (BSD License). Please remember that it is strongly coupled to on-going research and thus some parts are not fully mature yet. Furthermore, the code will also be subject to changes in the future which could include greater re-factoring of some parts.

Video: https://youtu.be/ZMAISVy-6ao

Papers:
* http://dx.doi.org/10.3929/ethz-a-010566547 (IROS 2015)
* http://dx.doi.org/10.1177/0278364917728574 (IJRR 2017)

Please also have a look at the wiki: https://github.com/ethz-asl/rovio/wiki

### Install without opengl scene ###
Dependencies:
* ros
* kindr (https://github.com/ethz-asl/kindr)
* lightweight_filtering (as submodule, use "git submodule update --init --recursive")

```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install with opengl scene ###
Additional dependencies: opengl, glut, glew (sudo apt-get install freeglut3-dev, sudo apt-get install libglew-dev)
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

### Euroc Datasets ###
The rovio_node.launch file loads parameters such that ROVIO runs properly on the Euroc datasets. The datasets are available under:
http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

### Further notes ###
* Camera matrix and distortion parameters should be provided by a yaml file or loaded through rosparam
* The cfg/rovio.info provides most parameters for rovio. The camera extrinsics qCM (quaternion from IMU to camera frame, Hamilton-convention) and MrMC (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
* Especially for application with little motion fixing the IMU-camera extrinsics can be beneficial. This can be done by setting the parameter doVECalibration to false. Please be carefull that the overall robustness and accuracy can be very sensitive to bad extrinsic calibrations.
