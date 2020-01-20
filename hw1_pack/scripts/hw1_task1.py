#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
##from nav_msgs.msg import Odometry
VEL_X = 0.3

class RoboDriver:
    def __init__(self):
        self.vel_msg = Twist()
        self.rate = rospy.Rate(1)
        self.cmb_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 0)
        ##self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

    def move_forward(self, dis):
        start_time = rospy.Time.now().to_sec()
        moved_dis = 0
        self.vel_msg.linear.x = VEL_X
        self.cmd_pub.publish(vel_msg)
        while moved_dis < dis:
          now_time = rospy.Time.now().to_sec()
          dt = now_time - start_time
          moved_dis += dt * self.vel_x
        self.vel_msg.linear.x = 0
        self.cmd_pub.publish(vel_msg)

    ##def odom_callback(self, msg):
        ##print msg

    def main(self):
        while not rospy.is_shutdown():
            self.move_forward(1)
            self.rate.sleep() 

if __name__ == '__main__':
    rospy.init_node("cmd_node")
    r = RoboDriver()
    r.main()
