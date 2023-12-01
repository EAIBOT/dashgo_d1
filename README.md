# The dashgo_d1 package for Ubuntu 20.04 and ROS noetic

## Description
Dashgo D1 

Ubuntu 20.04 ROS noetic

Version:v3.0

Date:2023-8-28

### Installation
```
$ mkdir ~/dasgho_ws/src -p
$ cd ~/dashgo_ws/src
$ git clone https://github.com/m11112089/dashgo_d1.git
$ cd ~/dashgo_ws
$ catkin_make
$ source devel/setup.bash
```
### Dependency
```
$ pip install pyserial
```
### Run dashgo driver

First make the node executable:
```
$ sudo chmod 777 ~/dashgo_ws/src/dashgo_d1/dashgo_driver/nodes/dashgo_driver.py
```

And run the demo
```
$ roslaunch dashgo_driver demo.launch
```

In order to control the platform with keyboard teleop, install it with
```
$ sudo apt install ros-noetic-teleop-twist-keyboard
```
and run
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


## Changes list
1. In python3, the return value of map() is no longer list, but iterators, you have to convert iterators to list.

2. Python3's string is Unicode, python2.7 is ascii, so we need to encode the string sent to Arduino into ascii, and decode the string received from Arduino into Unicode.

3. thread should be changed to _thread (Low-level threading API).

4. Python 3 disallows mixing the use of tabs and spaces for indentation.

5. Delete yocs_velocity_smoother and change from smoother_cmd_vel to cmd_vel.

6. Python3 print need to add parentheses.
___
Please note that this fork is not officially endorsed by [EAIBOT/dashgo_d1]. While we strive to provide a robust solution for Ubuntu 20.04, we recommend users consult the official documentation and support channels for the most up-to-date information.
