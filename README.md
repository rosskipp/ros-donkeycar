# ROS Joystick Controlled Donkeycar

### Materials
- Some sort of donkeycar https://docs.donkeycar.com/guide/build_hardware/
- Raspberry Pi 4

### Setup
- Build out the Donkeycar as you normally would, but just install a fresh Ubuntu OS
- Install ROS (noetic) http://wiki.ros.org/Installation/Ubuntu
- Install the i2c_pwm package to be able to control the servos/motor with ROS https://gitlab.com/bradanlane/ros-i2cpwmboard
- I needed to alter file permissions/groups to get i2c working for my ubuntu user. See this: https://lexruee.ch/setting-i2c-permissions-for-non-root-users.html
- Install the joystick control package http://wiki.ros.org/joy
- Clone this repo into catkin workspace src directory
- Run the launch file from this repo.
