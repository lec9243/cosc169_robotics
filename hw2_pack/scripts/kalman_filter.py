#!/usr/bin/env python
import rospy as rsp
import numpy as np
import matplotlib.pyplot as plt
import tf
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
_RATE = 10
_DT = 0.1


class KalmanFilter:
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None):
        self.n = F.shape[0]
        self.l = B.shape[0]
        self.k = H.shape[0]
        self.u = np.zeros((self.k, 1))
        self.z = np.zeros((self.l, 1))
        self.x = np.zeros((self.n, 1))
        self.xp = np.zeros((self.n, 1))
        self.F = F
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P

    def predict(self):
        self.xp = np.dot(self.F, self.x) + np.dot(self.B, self.u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self):
        z1 = np.dot(self.H, self.x)
        r = self.z - z1
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), (1/S))
        self.x = self.xp + np.dot(K, r)
        self.P = self.P - np.dot(np.dot(np.dot(np.dot(self.P, self.H.T), (1/S)),self.H), self.P)

        tf_bc = tf.TransformBroadcaster()
        tf_bc.sendTransform((self.x, 0, 0), (0, 0, 0, 1), rsp.Time.now(), "base_footprint.", "odom_kf")

class RobotDriver:
    def __init__(self, F, B, H, Q, R, P):
        self.flag_ = True
        self.moving_ = False
        self.init_pose = None
        self.rate = rsp.Rate(_RATE)
        self.kalman = KalmanFilter(F,B,H,Q,R,P)
        self.kf_info = PoseWithCovarianceStamped()
        self.pose_pub = rsp.Publisher('kf_pose', PoseWithCovarianceStamped, queue_size = 0)
        #self.cmd_sub = rsp.Subscriber('cmd_vel', Twist, self.cmd_callback)
        self.init_sub = rsp.Subscriber("initialpose", PoseWithCovarianceStamped, self.init_callback)
        self.pose_sub = rsp.Subscriber("pose", PoseStamped, self.pose_callback)
        self.scan_sub = rsp.Subscriber("scan", LaserScan, self.scan_callback)
        self.count = 0
        self.t = []
        self.x = []
        self.p = []

    def cmd_callback(self, msg):
        #print("cmd")
        if msg.linear.x > 0:
            self.moving_ = True
        else:
            self.moving_ = False
        self.kalman.u = msg.linear.x

    def init_callback(self, msg):
        #print("init")
        if self.flag_:
            print("change flag")
            self.kalman.x = msg.pose.pose.position.x
            self.flag_ = not self.flag_
        else:
            self.kalman.x = self.kalman.x

    def pose_callback(self, msg):
        #print("pose")
        if not self.moving_:
            self.moving_ = not self.moving_
            self.init_pose = msg.pose.position.x
        self.kalman.u = (msg.pose.position.x - self.init_pose)/0.1
        self.init_pose = msg.pose.position.x


    def scan_callback(self, msg):
        if (not np.isinf(msg.ranges[0]) and not np.isnan(msg.ranges[0])):
            self.kalman.z = 2 - msg.ranges[0]
        else:
            self.kalman.z = self.kalman.z

    def main(self):
        while not rsp.is_shutdown():
            if self.flag_:
                continue
            elif self.moving_:
                self.count += 1
                self.kalman.predict()
                self.kalman.update()
                self.x.append(self.kalman.x) ##for cmd_vel
                self.t.append(self.count * _DT)  ##for cmd_vel
                self.p.append(self.init_pose)
                self.kf_info.pose.pose.position.x = self.kalman.x
                self.kf_info.pose.covariance[0] = self.kalman.P
            else:
                if self.count == 0:
                    continue
                else:
                     break
            self.pose_pub.publish(self.kf_info)
            self.rate.sleep()
        print (self.x)
        print (self.t)
        print (self.p)
        plt.plot(self.t, self.x)
        plt.plot(self.t, self.p)
        plt.show()


if __name__ == '__main__':
    rsp.init_node("kf_node")
    F = np.array([1])
    B = np.array([_DT])
    H = np.array([1])
    Q = np.array([1])
    P = np.array([1000])
    R = np.array([1])
    r = RobotDriver(F, B, H, Q, R, P)
    r.main()
