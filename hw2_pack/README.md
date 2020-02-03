This package contains two scripts for task1 and task3 and one launch file for task2. To use this package, put it under the catkin workspace and run the following command.
```
$ catkin_make
$ source devel/setup.bash
$ roslaunch hw2_pack kalman_filter.launch
```
open new terminal run
```
$ rosbag play recordall.bag
```

To run depthimage_to_laserscan version, open new terminal and run 
```
$ roslaunch hw2_pack depth_scan.launch
```

Note there are something need to chamge by hand in scripts, please see comments
