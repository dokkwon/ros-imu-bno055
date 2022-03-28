# ROS driver for IMU Bosch BNO055 (I2C)

This is a ROS node for the BNO055 IMU that communicates via I2C and without any dependencies besides libi2c-dev. It does **not** require RTIMULib, RTIMULib2, RTIMULib3 or whatever the latest sequel is. It is specifically targeted at using a BNO055 with Texas Instruments SK board, or any other board that has native I2C. Please note that this ROS node has been forked from https://github.com/dheera/ros-imu-bno055 and the calibration routine has been added on top of it.

The BNO055 supports I2C and UART communication. This driver supports I2C only. If you are looking for a UART driver, see [RoboticArts/ros_imu_bno055](https://github.com/RoboticArts/ros_imu_bno055) instead.

## IMU Calibration

It is recommended to calibrate the BNO055 IMU before using it for the applications such as localization, SLAM, navigation, etc. We can start calibration by the following command:

```shell
roslaunch imu_bno055 imu_calib.launch
```

In order to calibrate the BNO055 IMU more effectively, it is recommended to follow the guideline below. Once the calibration statuses of all sensors are 3 (i.e., fully calibrated), the calibration process is completed and the calibration profile is saved in a file so that it can be reused.

1. Accelerometer Calibration
    * Place the device in 6 different stable positions for a period of few seconds to allow the accelerometer to calibrate.
    * Make sure that there is slow movement between 2 stable positions.
    * The 6 stable positions could be in any direction, but make sure that the device is lying at least once perpendicular to the x, y and z axis.
    * The register CALIB_STAT is read to see the calibration status of the accelerometer.

2. Gyroscope Calibration
    * Place the device in a single stable position for a period of few seconds to allow the gyroscope to calibrate.
    * The register CALIB_STAT is read to see the calibration status of the gyroscope.

3. Magnetometer Calibration
    * Make some random movements (for example: writing the number ‘8’ on air) 
    * The register CALIB_STAT is read to see the calibration status of the magnetometer.


The information on BNO055's operation modes and the calibration process can be found in Sec.3.3 and Sec.3.10, respectively, in [BNO055 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf).

### Parameters

The followings are the parameters configured in imu_calib.launch.

* **device** -- the path to the i2c device. Default is /dev/i2c-10. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
* **address** -- the i2c address of the IMU. Default is 0x28.
* **operation_mode** -- IMU operation mode. Only IMU mode and NDOF mode are supported.
* **calib_file_name** -- Calibration profile is saved in this file if the calibration process is successful. 


## Deploying IMU

We can deploy the calibrated BNO055 IMU by the following command:

```shell
roslaunch imu_bno055 imu.launch
```

### Parameters

The followings are the parameters configured in imu.launch.

* **device** -- the path to the i2c device. Default is /dev/i2c-10. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
* **address** -- the i2c address of the IMU. Default is 0x28.
* **frame_id** -- frame_id in ROS topics.
* **operation_mode** -- IMU operation mode. Only IMU mode and NDOF mode are supported.
* **calib_file_name** -- File name where the calibration profile is saved by the calibration process.

### Outputs topics
* **/data** (sensor\_msgs/Imu) -- fused IMU data
* **/raw** (sensor\_msgs/Imu) -- raw accelerometer data
* **/mag** (sensor\_msgs/MagneticField) -- raw magnetic field data
* **/temp** (sensor\_msgs/Temperature) -- temperature data
* **/status** (diagnostic\_msgs/DiagnosticStatus) -- a DiagnosticStatus object showing the current calibration, interrupt, system status of the IMU


## Service call
* **/reset** (std\_srvs/Trigger) -- resets the IMU

# Usage notes

## Texas Instruments SK Board

We can launch this ROS node in the ROS1 Docker container on the SK board using Robotics SDK. Please refer to [Robotics SDK User Guide](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/08_01_00/docs/index.html) to set up the Robotics SDK Docker container environment on the SK board. 

Make sure that the GPIO header is enabled and the BNO055 IMU is connected properly. Then run the following commands to launch the IMU sensor:

```shell
root@j7-evm:~$ j7ros_home/docker_run_ros1.sh
root@j7-docker:~/j7ros_home/ros_ws$ catkin_make --source /opt/ros-imu-bno055
root@j7-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
root@j7-docker:~/j7ros_home/ros_ws$ roslaunch imu_bno055 imu.launch
```


