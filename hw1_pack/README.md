This package contains two scripts for task1 and task3 and one launch file for task2. To use this package, put it under the catkin workspace and run the following command.
```
$ catkin_make
$ source devel/setup.bash
$ roslaunch hw1_pack hw1_task2.launch
```

To run scripts indivually, first run roscore and serial_bridge.sh, then run the command
```
rosrun hw1_pack hw1_task*.py
```
