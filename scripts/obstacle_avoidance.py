#!/usr/bin/env python3
import sys, os

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

import numpy as np

from copy import deepcopy
from collections import deque

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))


class obstacle_avoidance:
    def __init__(self, start_dist = 0.4, linear_gain = 0.000005, turn_gain = 0.00025):
        rospy.init_node('obstacle_avoidance')
        
        ## Use simulation time (i.e. get time from rostopic /clock)
        rospy.set_param('use_sim_time', 'true')
        rate = rospy.Rate(10)
        while rospy.Time.now() == rospy.Time(0):
            rate.sleep()

        ## Initial state
        self.scans = deque()
        self.start_dist = start_dist
        self.gain = linear_gain
        self.turn_gain = turn_gain
        self.V = 0.0
        self.om = 0.0
        self.seq = 0

        ## Set up publishers and subscribers
        #self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.repulsion_publisher = rospy.Publisher("obstacle_repulsion", Twist, queue_size=10)
        self.repulsion_vis_publisher = rospy.Publisher("obstacle_repulsion_vis", TwistStamped, queue_size=10)

    def scan_callback(self, msg):
        #print('got new scan')
        self.scans.append((msg.header.stamp,
                        np.array([i*msg.angle_increment + msg.angle_min for i in range(len(msg.ranges))]),
                        np.array(msg.ranges)))
    
    def cmd_vel_callback(self, msg):
        self.V = msg.linear.x
        self.om = msg.angular.z


    def calculate_repulsion(self):
        
        latest_scan_raw = self.scans[-1][-1]
        #print(latest_scan_raw[0])
        scan = [[(ind / 360) * 2 * np.pi, latest_scan_raw[ind]] for ind in range(len(latest_scan_raw))]
        if self.V > 0:
            scan = scan[0:90] + scan[270:359]
        else:
            scan = scan[90:270]
        force_points_polar = [point for point in scan if point[1] < self.start_dist] # polar points that are under threshold
        if len(force_points_polar) == 0:
            force_vec_sum = [0,0]
        else:
            force_vecs_polar = [[point[0], -1 * np.sign(point[1]) * 1/((point[1])**2)] for point in force_points_polar] # convert to force vectors pointing away from points
            force_vecs_cartesian = [[point[1]*np.cos(point[0]), point[1]*np.sin(point[0])] for point in force_vecs_polar] # cartesian points
            force_vec_sum = np.sum(force_vecs_cartesian, axis=0)
        
        #Now convert this vector to control inputs
        
        rep_msg = Twist()
        rep_msg.linear.x =  self.gain * force_vec_sum[0]
        rep_msg.angular.z = np.sign(self.V)/(1 + abs(self.V)) * self.turn_gain * force_vec_sum[1] # The faster we go, the more impact a small turn will have on lateral translation
        self.repulsion_publisher.publish(rep_msg)

        rep_vis = TwistStamped()
        rep_vis.twist = rep_msg
        rep_vis.twist.angular.z = rep_vis.twist.angular.z * 5
        rep_header = Header()
        rep_header.seq = self.seq
        self.seq += 1
        rep_header.stamp = rospy.get_rostime()
        rep_header.frame_id = "base_footprint"
        rep_vis.header = rep_header

        self.repulsion_vis_publisher.publish(rep_vis)
        
        return rep_msg

    
    def run(self):
        #print("running")
        rate = rospy.Rate(150)
        while True:
            rate.sleep()
            if not self.scans:
                rate.sleep()
                continue
            while len(self.scans) > 1:    # keep only the last element in the queue, if we're falling behind
                self.scans.popleft()
            #print('bla')
            self.calculate_repulsion()
            
            
            #print(rep)
            #print(len(self.scans[-1][-1]))

if __name__ == '__main__':
    avoid = obstacle_avoidance()
    avoid.run()