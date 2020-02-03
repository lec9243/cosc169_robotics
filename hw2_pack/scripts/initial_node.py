#!/usr/bin/env python
import rospy as rsp
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
_RATE = 10
_DT = 0.1

rsp.init_node("initial_node")
rate = rsp.Rate(_RATE)
init_pub = rsp.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 0)
init_info = PoseWithCovarianceStamped()

if __name__ == "__main__":
    while not rsp.is_shutdown():
        init_pub.publish(init_info)
        rate.sleep() 
