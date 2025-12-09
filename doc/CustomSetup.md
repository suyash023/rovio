# Running ROVIO on a custom camera IMU sensor setup #

### Hardware ###
ROVIO is a Visual Inertial odometry algorithm, primarily designed to work with a camera and IMU module.
Sensors:
* IMU - required. You can use IMU of any grade. Consumer grade IMUs are usually low-cost, and can provide good accuracy if good calibration data is available.
* Camera - required. Global Shutter cameras are preferred, but they are less in number and usually a little more expensive than Rolling shutter cameras. Rolling shutter cameras can also be used, but expect some degradation in accuracy and also filter blowing up easily.
* GPS - optional.
* Optical Flow/ Velocity sensors - optional.

### Timestamping and clock expectations ###
Each input sensor stream within ROVIO is expected to be timestamped with the same clock source.
It is best practice to timestamp the data as soon as the first fragment is obtained within the sensor driver.
If the sensor streams come from different chipsets then make sure there is a time synchronization mechanism in place,
or a mechanism to keep track of the time-offsets between the clocks.

Another aspect of the time synchronization mechanism will be covered in the extrinsics calibration section.

### Calibration ###

Intrinsics - Specific and internal to the sensor.
Extrinsics - Between two sensors.

* Intrinsics calibration:
    * IMU -  calibration parameters typically consist of random walk noise parameters. Obtained from the allan calibration framework. See this link.
    * Camera - Calibration parameters consist of focal length (fx, fy) and lens distortion parameters. ROVIO currently supports the following camera models:
        * Radtan (plumb_bob) model : for non-fish eye models . Typically lenses with camera fov < 90 degrees.
        * Equidistant model (fisheye) : for fisheye models. Typically camera lenses with camera fov > 90 degrees.
        * Double sphere model : for high order fish eye camera lense. Typically lenses with > 180 degrees.
    * Calibration tools:
        * IMU : (allan calibration)[link], (allan calibration from rpng ) [link]
        * Camera : (ros2 calibration tool)[link], (opencv camera calibration)[link]

File formatting:
The extrinsics calibration from kalibr tool and possibly other tools is usually expressed as a transformation matrix.
ROVIO expects a quaternion from camera to IMU and a translation vector from IMU to camera, hence the transformation matrix output from kalibr would have to be converted to quternion and translation. You can use (this)[link] tool for the conversion.
Modify the file (info)[rovio/cfg/rovio.info] from line numbers 11-18. Note that the quaternion has to be rotation from camera to IMU.

* Extrinsics calibration:
    * IMU - camera : Relative placement of IMU camera sensor expressed as a transform. Also computed is an addtional time-offset between the timestamps of the two sensors, which is asssumed to be static.
    * IMU - other sensor : Relative Placement of IMU and any other sensor expressed as a transform.
    * Calibration tools:
        * IMU - camera : (kalibr calibration)[link]
        * IMU - other : Transform might have to be obtained based on guess work or CAD values.
          File formatting:
          In file euroc_cam0.yaml replace the data row on line number 8 (under camera matrix) with the fx, fy, cx,cy values (focal lengths and optical center) values from the intrinsics calibration.


