#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

class RoboDriver:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/pose', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        print msg

    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep() 

if __name__ == '__main__':
    rospy.init_node("odom_node")
    r = RoboDriver()
    r.main()
