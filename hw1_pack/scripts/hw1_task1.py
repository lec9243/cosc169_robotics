#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
##from nav_msgs.msg import Odometry

class RoboDriver:
    def __init__(self):
        self.vel_x = 0.1
        self.moved_dis = 0
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 0)
        ##self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

    def move_forward(self, dis):
        start_time = rospy.Time.now().to_sec()
        while self.moved_dis < dis:
          self.cmd_pub.publish(self.vel_msg)
          now_time = rospy.Time.now().to_sec()
          print (self.moved_dis)
          dt = now_time - start_time
          self.moved_dis = dt * self.vel_x
          self.rate.sleep()

    ##def odom_callback(self, msg):
        ##print msg

    def main(self):
        while not rospy.is_shutdown():
            self.vel_msg.linear.x = self.vel_x
            # self.cmd_pub.publish(self.vel_msg)
            self.move_forward(1)
            self.vel_msg.linear.x = 0
            self.cmd_pub.publish(self.vel_msg)
            self.rate.sleep() 

if __name__ == '__main__':
    rospy.init_node("move_forward")
    r = RoboDriver()
    r.main()
