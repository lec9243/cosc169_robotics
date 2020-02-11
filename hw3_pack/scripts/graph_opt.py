#!/usr/bin/env python
import rospy as rsp
import numpy as np
import matplotlib.pyplot as plt
import g2o
import tf
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
_DT = 0.1

### Template
class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super(PoseGraphOptimization, self).__init__()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super(PoseGraphOptimization, self).set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super(PoseGraphOptimization, self).initialize_optimization()
        super(PoseGraphOptimization, self).optimize(max_iterations)

    def add_vertex(self, id, pose, is_landmark, fixed=False):
        if is_landmark:
            v_se2 = g2o.VertexPointXY()
        else:
            v_se2 = g2o.VertexSE2()
        v_se2.set_id(id)
        v_se2.set_estimate(pose)
        v_se2.set_fixed(fixed)
        super(PoseGraphOptimization, self).add_vertex(v_se2)

    def add_edge(self, vertices, measurement, is_between_pose,
            robust_kernel=None):
        if is_between_pose:
            information = np.identity(3)
            edge = g2o.EdgeSE2() # between pose nodes
        else:
            information=np.identity(2)
            edge = g2o.EdgeSE2PointXY() # between pose node and landmark node
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)  # relative pose
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super(PoseGraphOptimization, self).add_edge(edge)

    def get_pose(self, id):
        return self.vertex(id).estimate()

## Class that will use optimize
class GODriver:
    def __init__(self):
        rsp.init_node("graph_node")
        ## GraphOptimizer
        self.graph_opt = PoseGraphOptimization()
        ## Subscribers
        self.cmd_sub = rsp.Subscriber("cmd_vel", Twist, self.cmd_callback)
        self.pose_sub = rsp.Subscriber("pose", PoseStamped, self.pose_callback)
        self.scan_sub = rsp.Subscriber("scan", LaserScan, self.scan_callback)
        ## Landmark
        self.id = 0
        self.landmark = self.graph_opt.add_vertex(self.id, [2,0], True)
        self.last_pose = [-1,-1,0]
        self.dist_ = 0
        self.flag = False
        self.count = 0
        ## For Plot
        self.t = 0.0
        self.t_lst = []
        self.x_lst = []
        self.y_lst = []
        self.x_grd_lst = []
        self.y_grd_lst = []
        self.th_grd_lst = []
        self.th_lst = []
        self.y_op_lst = []
        self.x_op_lst = []
        self.th_op_lst = []

### Callback function for pose, add vertex and edges between pose and pose and pose and node here.
    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        rotation = (msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w)
        euler_angle = tf.transformations.euler_from_quaternion(rotation)
        th = euler_angle[2]
        if ((x != self.last_pose[0]) and (y != self.last_pose[1])):
            self.id += 1
            pose_ = g2o.SE2(x, y, th)
            self.graph_opt.add_vertex(self.id, pose_, False)
            ms = [(x-self.last_pose[0]),(y-self.last_pose[1]),(th-self.last_pose[2])]
            measure = g2o.SE2(ms[0],ms[1],ms[2])
            if (id > 1):
                self.graph_opt.add_edge([self.id, self.id-1], measure, True)
                self.graph_opt.add_edge([self.id, 0], [self.dist_, 0], False)
            self.last_pose = [x,y,th]
            self.t += 0.1
            self.t_lst.append(self.t)
            self.x_lst.append(x)
            self.y_lst.append(y)
            self.th_lst.append(th)

### Callback function for scan, get distance between robot and landmark
    def scan_callback(self, msg):
           for i in range(5):
               if msg.ranges[i] < self.dist_ :
                   self.dist_ = msg.ranges[i]

### Callback function for cmd, use velocity as flag to stop and do plot
    def cmd_callback(self, msg):
        if msg.linear.x == 0:
            if self.flag:
                self.count = 1
            else: self.count = 0
        else: self.flag = True

### Main function, acutllay do plot and call opotimization.
    def main(self):
        while (self.count == 0):
            print(self.count)
            continue
        if (self.count > 0):
            self.graph_opt.save("graph_init.g2o")
            print("after save")
            self.graph_opt.optimize()
            print("after optimize")
            self.graph_opt.save("graph_opt.g2o")
            print("after save optimize")
        for i in range(1,self.id):
            new_p = self.graph_opt.get_pose(i).translation()
            new_th = self.graph_opt.get_pose(i).rotation().angle()
            ##print (new_th)
            self.x_op_lst.append(new_p[0])
            self.y_op_lst.append(new_p[1])
            self.th_op_lst.append(new_th)
            self.x_grd_lst.append(1)
            self.y_grd_lst.append(0)
            self.th_grd_lst.append(0)
        plt.figure()
        plt.plot(self.t_lst[0:50], self.x_lst[0:50], 'r-', label = "old_x")
        plt.plot(self.t_lst[0:50], self.x_op_lst[0:50], 'g-', label = "optimized_x")
        plt.plot(self.t_lst[0:50], self.x_grd_lst[0:50], 'b-', label = "ground_x")
        plt.xlabel("time")
        plt.ylabel("x distance")
        plt.title("x vs t")
        plt.legend()
        ## plt.show()
        plt.savefig("x_t.png")
        plt.figure()
        plt.plot(self.t_lst[0:50], self.y_lst[0:50], 'r-', label = "old_y")
        plt.plot(self.t_lst[0:50], self.y_op_lst[0:50], 'g-', label = "optimized_y")
        plt.plot(self.t_lst[0:50], self.y_grd_lst[0:50], 'b-', label = "ground_y")
        plt.xlabel("time")
        plt.ylabel("y distance")
        plt.title("y vs t")
        plt.legend()
        ## plt.show()
        plt.savefig("y_t.png")
        plt.figure()
        plt.plot(self.t_lst[0:50], self.th_lst[0:50], 'r-', label = "old_th")
        plt.plot(self.t_lst[0:50], self.th_op_lst[0:50], 'g-', label = "optimized_th")
        plt.plot(self.t_lst[0:50], self.th_grd_lst[0:50], 'b-', label = "ground_th")
        plt.xlabel("time")
        plt.ylabel("theta value")
        plt.title("theta vs t")
        plt.legend()
        plt.savefig("th_t.png")
        plt.show()
        rsp.spin()


if __name__ == '__main__':
    drive = GODriver()
    drive.main()
