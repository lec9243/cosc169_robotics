#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class RoboDriver:
    def __init__(self):
        self.mk_msg = Marker()
        self.rate = rospy.Rate(10)
        self.odom_sub = rospy.Subscriber('/pose', PoseStamped, self.odom_callback)
        self.mark_pub = rospy.Publisher('markers', Marker, queue_size = 0)

    def odom_callback(self, msg):
        #print msg
        self.mk_msg.header.frame_id = msg.header.frame_id
        self.mk_msg.header.seq = msg.header.seq
        self.mk_msg.pose = msg.pose
        self.mk_msg.color.a = 1
        self.mk_msg.color.r = 1
        self.mk_msg.color.b = 1
        self.mk_msg.scale.x = 0.5;
        self.mk_msg.scale.y = 0.5;
        self.mk_msg.scale.z = 0.5;
        #self.mk_msg.pose.position.z = 0

    def main(self):
        while not rospy.is_shutdown():
            self.mark_pub.publish(self.mk_msg)
            self.rate.sleep() 

if __name__ == '__main__':
    rospy.init_node("get_position")
    r = RoboDriver()
    r.main()
