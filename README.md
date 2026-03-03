# Franka Docker

This dockerfile is intended as a general purpose dockerfile for the fr3 robot in the lab.

It has the following libraries installed:
- `libfranka`, the low level c++ library for controlling the Franka
- `pylibfranka`, python bindings for low level control of the Franka
- `franky`, a python library for high level control of the Franka
- `panda-py`, a python library that can be used to activate/lock/unlock the franka remotely.

Additionally, it contains the following

- ROS2, for common visualization tools like rqt and rviz2
- The realsense-ros2 package, for easy use of our realsense cameras
- A ROS2 node, franky_bridge that acts as a minimal wrapper over franky, to allow for controlling the robot at a high level by publishing to topics, or to read robot state data from ROS topics.

If you prefer functionality similar to the Kinova, where all you need is a python class to talk to the robot, it is best to use the `franky` library directly, and only use ROS for communicating and visualization of data. Or, create a python class that just publishes to the franky node.

TODOs:
- Bring in franka description urdf for better TF tree publishing
- Testing on a lot of interactions in the ROS2 node (e.g. interactions between calling blocking and non blocking funcs in succession)


## Using the Franka
1. Power on the Franka
2. Go to 192.168.1.105/desk to access the UI, but don't take control unless you need to
3. Build repo with `colcon build --symlink-install` in the `franka_ws` folder. Then `source install/setup.bash`.
4. To get control over code, use the `enable_fci.py` script.
5. From there, you can start the ros node with `ros2 launch franky_ros franky_bringup.launch.py`
6. To power off Franka, use desk.


## NOTES
TODO: fix for robotvision and odin, not just loki

once inside docker container
```bash
cd /home/user/intervention-learning
```


to build you need to 
```bash
colcon build --symlink-install
souce install/setup.bash
```

the symlink thing is needed for ros2 to actually update stuff

there is test scripts which you need to unlock for the robot to work. NOTE: make sure to lock when shutting down.
```bash
python3 ~/intervention-learning/test_scripts/enable_fci.py
```

to run anything for franka you will probably need to run this

```bash
ros2 launch franky_ros franky_bringup.launch.py
```

running test node
```bash
ros2 run franky_ros franky_test
```


running xbox control node

first find the event of the xbox
```bash
cat /proc/bus/input/devices | grep -B 5 -A 5 "Microsoft X-Box"
```
update the compose so that it passes this event through
make sure the event has 
```bash
crw-rw-rw-  1 root  995 13, 84 Jan 14 00:15 event20
```
you can see that the ros sees the device with 
```bash
ros2 run joy joy_enumerate_devices
```

```bash
ros2 launch franky_ros franky_bringup.launch.py
```
