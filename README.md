# Pepper Controller Server

This package implements a service for a controller interface to control the pepper robot. It takes both the position and the stiffness from the service and is used by the modified [`naoqi_dcm_driver`](https://github.com/souljaboy764/naoqi_dcm_driver) to set the joint positions and the ["stiffness"](http://doc.aldebaran.com/2-5/naoqi/motion/control-stiffness.html) of Pepper.

## Running on the robot

In the contorller configuration file from [`pepper_dcm_robot`](https://github.com/ros-naoqi/pepper_dcm_robot), include the launch file from this package instead of the one from [`pepper_control`](https://github.com/ros-naoqi/pepper_virtual) and change the rosparam yaml file that gets loaded to the one from this package in the [`config`](config) directory.

## TODO

- Test Compliance controller
- Feedback Linearization controller
