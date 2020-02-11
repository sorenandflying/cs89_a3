#! /usr/bin/env python

import rospy
import numpy as np
# import g2o
import math

# from pose_graph_opt import PoseGraphOptimization as PGO

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan

class SLAM:
    def __init__(self):
        self.slam_file = open("slam2d.g2o", "w")
        self.last_pose = Pose()
        self.last_pose_time = 0
        self.last_pose_vtx = -1
        self.scan_flag = False
        self.scan_vertex = -1
        self.scan_time = 0
        self.scan_dist = 0
        self.vertex_count = 0

    def next_id(self):
        id = self.last_pose_vtx + 1
        self.last_pose_vtx += 1
        return id

    def file_pose_vtx(self, x, y, theta):
        vtx_id = self.next_id()
        self.slam_file.write("VERTEX_SE2 " + str(vtx_id) + " " + str(x) + " " + str(y) + " " + str(theta) + "\n")
        return vtx_id

    def file_landmark_vtx(self, x, y):
        self.slam_file.write("VERTEX_XY " + str(self.scan_vertex) + " " + str(x) + " " + str(y) + "\n")

    def file_pose_edge(self, id1, id2, dx, dy, info_mtx):
        if id1 < 0 or id2 < 0:
            return
        dtheta = 0
        self.slam_file.write("EDGE_SE2 " + str(id1) + " " + str(id2) + " " + str(dx) + " " + str(dy) + " " + str(dtheta))
        for i in range(6):
            self.slam_file.write(" " + str(info_mtx.item(i)))
        self.slam_file.write("\n")

    def file_landmark_edge(self, id1, id2):
        self.slam_file.write("EDGE_SE2_XY " + str(id1) + " " + str(id2) + "\n")

    def simple_pose_callback(self, pose_info):
        curr_x = pose_info.pose.position.x
        curr_y = pose_info.pose.position.y
        curr_t = pose_info.header.stamp.secs

        last_x = self.last_pose.position.x
        last_y = self.last_pose.position.y
        last_t = self.last_pose_time
        last_idx = self.last_pose_vtx

        # new_idx = self.file_pose_vtx(curr_x, curr_y, 0)
        new_idx = self.file_pose_vtx(0, 0, 0)
        dx = curr_x - last_x
        dy = curr_y - last_y
        info_mtx = np.matrix('0.9,0.9;0.9,0.9;0.9,0.9')
        self.file_pose_edge(last_idx, new_idx, dx, dy, info_mtx)

        self.last_pose = pose_info.pose

    def pose_callback(self, pose_info):
        # Check for scan flag
        # If there is a flag:
            # Interpolate pose @ scan time
            # Add pose vertex
            # Add pose -> pose edge
            # Add pose -> landmark edge
        curr_x = pose_info.pose.position.x
        curr_y = pose_info.pose.position.y
        curr_t = pose_info.header.stamp.secs

        last_x = self.last_pose.position.x
        last_y = self.last_pose.position.y
        last_t = self.last_pose_time

        if (self.scan_flag):
            # Interpolate scan position based on time
            t0 = self.scan_time - last_t
            t1 = curr_t = curr_t - self.scan_time
            t_total = curr_t - last_t

            scan_x = (last_x * t1 + curr_x * t0) / t_total
            scan_y = (last_y * t1 + curr_y * t0) / t_total

            pose_idx = self.file_pose_vtx(scan_x, scan_y, 0)
            self.file_landmark_edge(pose_idx, self.scan_vertex)

        # Update last logged pose and time
        self.last_pose = pose_info.pose
        self.last_time = pose_info.header.stamp.secs

    def scan_callback(self, scan_info):
        if self.scan_vertex < 0:
            self.scan_vertex = self.next_id()

        # Set scan flag
        self.scan_flag = True
        # Save scanned distance
        goal_angle = math.pi
        laser_idx = int(math.floor((goal_angle - scan_info.angle_min) / scan_info.angle_increment))
        # Save scan information
        self.scan_dist = scan_info.ranges[laser_idx]
        rospy.loginfo(self.scan_dist)
        self.scan_time = scan_info.header.stamp.secs + laser_idx * scan_info.time_increment

    def close_file(self):
        rospy.loginfo("Closing slam file\n")
        self.slam_file.close()


# Subscribers
slam = SLAM()
if __name__ == "__main__":
    rospy.init_node('slam2d')
    pose_sub = rospy.Subscriber('pose', PoseStamped, slam.pose_callback)
    scan_sub = rospy.Subscriber('/scan', LaserScan, slam.scan_callback)
    rospy.spin()

slam.close_file()

