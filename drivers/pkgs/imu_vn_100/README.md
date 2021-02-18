[![Build Status](https://travis-ci.org/KumarRobotics/imu_vn_100.svg?branch=master)](https://travis-ci.org/KumarRobotics/imu\_vn\_100)

# imu\_vn\_100

![Picture of IMU](http://www.vectornav.com/images/default-source/products/vn-100-rugged.png?sfvrsn=2)

The `imu_vn_100` package is a linux ROS driver for VN-100 Rugged IMU of [VECTORNAV](http://www.vectornav.com/). The package is developed based on the official [SDK v0.3.2](http://www.vectornav.com/support/downloads) for Linux. The user manual for the device can be found [here](http://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/UM001.pdf?sfvrsn=10).

The package is tested on Ubuntu 14.04 with ROS indigo.

Note that the official SDK is modified within this package due to some bugs or unsupported features. Please carefully update the SDK, since the new SDK may not work with the provided software.

## License
* The license for the official SDK is the MIT license which is included in the `include/imu_vn_100/vncpplib`
* The license for the other codes is Apache 2.0 whenever not specified.

## Compiling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compiling a catkin package will work.

```
cd your_work_space
catkin_make --pkg imu_vn_100 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Example Usage

**Parameters**

`port` (`string`, `default: \tty\USB0`)

Port which the device connected to. This can be checked by command `dmesg`.

`frame_id` (`string`, `default: imu`)

The frame ID entry for the sent messages.

`baudrate` (`int`, `921600`)

The baud rate of the serial port. The available baud rates can be checked on the user manual. It is suggested that the baud rate is kept at `921600` to ensure the high frequency transmission. The device will send `permission denied` error code if the baud rate cannot support the desired data package at the desired frequency.

`imu_rate` (`int`, `200`)

The rate of the IMU data.

```
enable_magn  (bool, default: true)
enable_press (bool, default: true)
enable_temp  (bool, default: true)
```

Enable other possible messages that the driver is available to send. Note that the frequency of the data for each of these messages will be the same with the IMU data, if the topic is enabled.

`sync_rate` (`int`, `20`)

The rate of the sync out trigger signal. Note that the actual rate may not exactly match the provided rate, unless the provided rate is a divisor of 800. When `sync_rate` <= 0, it is disabled.

**Published Topics**

`imu/imu` (`sensor_msgs/Imu`)

The message contains the uncompensated (for the definition of UNCOMPENSATED, please refer to the [user manual](http://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/UM001.pdf?sfvrsn=10)) angular velocity and linear acceleration. Note that the orientation is not provided on this topic.

`imu/magnetic_field` (`sensor_msgs/MagneticField`)

Uncompensated magnetic field.

`imu/pressure` (`sensor_msgs/FluidPressure`)

Pressure.

`imu/temperature` (`sensor_msgs/Temperature`)

Temperature in degree Celsius

**Node**

With the provided launch file, do

```
roslaunch imu_vn_100 vn_100_cont.launch
```

## FAQ
1. The driver can't open my device?
Make sure you have ownership of the device in `/dev`.

2. Why I have permission error during the initialization process of the driver?
Most often, this is because the baud rate you set does not match the package size to be received. Try increase the baud rate.

## Bug Report

Prefer to open an issue. 

## Bugs or Features Required of the Official SDK

* [Solved] Define macros for options to set synchronization register and communication control register. (vndevice.h)

* [Solved] Pressure entry is not properly parsed and overwrites temperature in `vndevice_processAsyncData` when the async data output type is set to `VNASYNC_VNIMU`. (vndevice.c)

* [Solved] Synchronization time or count is not parsed in `vndevice_processAsyncData`. (vndevice.c)

* Angular velocity and linear acceleration are flipped when the device is set to binary output using `BG1_IMU`

* Orientation and uncompensated IMU measurements cannot be acquired within a single setting using async output.
