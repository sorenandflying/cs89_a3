#! /usr/bin/env python

import rospy
import numpy as np
import g2o
import math

from pose_graph_opt import PoseGraphOptimization

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan

class SLAM:
    def __init__(self):
        # File to write to
        self.slam_file = open("slam2d.g2o", "w")

        # Counter for current vertex
        self.vertex_count = 0

        # Landmark variables
        self.scan_flag = False
        self.first_scan_flag = True

        self.last_scan = LaserScan()
        self.curr_scan = LaserScan()

        self.last_scan_time = 0
        self.last_scan_dist = 0
        self.curr_scan_time = 0
        self.curr_scan_dist = 0

        self.scan_vertex = 0

        # Pose variables
        self.last_pose = PoseStamped()
        self.last_pose_time = 0
        self.last_pose_vtx = 0
        self.info_mtx = np.matrix('1, 0, 0, 1, 0, 1')

        # Graph and Pose optimization variables
        self.pgo = PoseGraphOptimization()

    # Generates and returns a new vertex id
    def next_id(self):
        id = self.last_pose_vtx + 1
        self.last_pose_vtx += 1
        return id

    #### Functions for saving graph data ####
    def save_pose_vtx(self, id, x, y, theta):
        self.slam_file.write("VERTEX_SE2 " + str(id) + " " + str(x) + " " + str(y) + " " + str(theta) + "\n")
        g2o_pose = g2o.SE2(x,y,theta)
        self.pgo.add_vertex(id, g2o_pose)

    def save_landmark_vtx(self, id, x, y):
        self.slam_file.write("VERTEX_XY " + str(id) + " " + str(x) + " " + str(y) + "\n")
        self.pgo.add_landmark_vertex(id, [x,y])

    def save_pose_edge(self, id1, id2, dx, dy, info_mtx):
        if id1 < 0 or id2 < 0:
            return
        dtheta = 0
        self.slam_file.write("EDGE_SE2 " + str(id1) + " " + str(id2) + " " + str(dx) + " " + str(dy) + " " + str(dtheta))
        for i in range(6):
            self.slam_file.write(" " + str(info_mtx.item(i)))
        self.slam_file.write("\n")
        new_info_mtx = np.identity(3)
        self.pgo.add_edge([id1, id2], g2o.SE2(dx, dy, dtheta), new_info_mtx)

    def save_landmark_edge(self, id1, id2):
        self.slam_file.write("EDGE_SE2_XY " + str(id1) + " " + str(id2) + "\n")
        self.pgo.add_landmark_edge([id1, id2])

    #### Callback functions ####
    def pose_callback(self, pose_info):
        curr_x = pose_info.pose.position.x
        curr_y = pose_info.pose.position.y
        # curr_t = pose_info.header.stamp.secs
        # rospy.loginfo("Pose Time: " + str(pose_info.header.stamp.secs))
        # rospy.loginfo(pose_info)
        curr_t = rospy.get_time()

        if self.scan_flag:
            last_x = self.last_pose.pose.position.x
            last_y = self.last_pose.pose.position.y
            last_t = self.last_pose_time

            # Interpolate scan position based on time
            t0 = self.curr_scan_time - last_t
            t1 = curr_t - self.curr_scan_time
            t_total = curr_t - last_t
            rospy.loginfo("\nLast T: " + str(last_t) + "\nCurr_T: " + str(curr_t) + "\ncurr_scan_time: " + str(self.curr_scan_time))
            if t_total == 0:
                return
            if t0 < 0 or t0 > t_total:
                rospy.loginfo("Bad t0: " + str(t0))
            if t1 < 0 or t1 > t_total:
                rospy.loginfo("Bad t1: " + str(t1))

            pose_x = (last_x * t1 + curr_x * t0) / t_total
            pose_y = (last_y * t1 + curr_y * t0) / t_total
            
            # Save pose at scan as a vertex
            pose_id = self.next_id()
            self.save_pose_vtx(pose_id, pose_x, pose_y, 0)

            # Check if this is the first vertex added to the graph
            if self.first_scan_flag:
                # self.scan_vertex = self.next_id()
                # If this is the first scan, save the landmark vertex
                self.save_landmark_vtx(self.scan_vertex, self.curr_scan_dist + pose_x, pose_y)
                self.first_scan_flag = False
            else:
                # Otherwise, create an edge between the last two poses based on change in scan distance
                dx = self.last_scan_dist - self.curr_scan_dist
                dy = 0
                self.save_pose_edge(self.last_pose_vtx, pose_id, dx, dy, self.info_mtx)
            # Create an edge between landmark and pose
            self.save_landmark_edge(pose_id, self.scan_vertex)

            # Update last pose vtx
            self.last_pose_vtx = pose_id
            # Set flag to false
            self.scan_flag = False

        self.last_pose = pose_info
        self.last_pose_time = curr_t

    def scan_callback(self, scan_info):
        # Save old scan info
        self.last_scan_dist = self.curr_scan_dist
        self.last_scan_time = self.curr_scan_time

        # Extract info from scan
        goal_angle = math.pi
        laser_idx = int(math.floor((goal_angle - scan_info.angle_min) / scan_info.angle_increment))
        self.curr_scan_dist = scan_info.ranges[laser_idx]
        # self.curr_scan_time = scan_info.header.stamp.secs + laser_idx * scan_info.time_increment
        self.curr_scan_time = rospy.get_time()

        self.scan_flag = True
        

    def close_file(self):
        rospy.loginfo("Closing slam file\n")
        self.slam_file.close()



# Main script
slam = SLAM()
if __name__ == "__main__":
    rospy.init_node('slam2d')
    pose_sub = rospy.Subscriber('pose', PoseStamped, slam.pose_callback)
    scan_sub = rospy.Subscriber('scan', LaserScan, slam.scan_callback)
    rospy.spin()

slam.pgo.save("slam2d_unoptimized.g2o")
slam.pgo.optimize(5)
slam.pgo.save("slam2d_optimized.g2o")
slam.close_file()



