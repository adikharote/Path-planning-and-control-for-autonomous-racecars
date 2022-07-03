#!/usr/bin/env python3

import rospy
from fsd_common_msgs.msg import Waypoint, WaypointsStamped, ConeOdom
from abc import ABCMeta
import math
import matplotlib.pyplot as plt
import time
import numpy as np
from fs_msgs.msg import Track

class MapHandle():
    __metaclass__ = ABCMeta

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln = plt.scatter(x=np.array([]), y=np.array([]),s = 1.0, c=np.array([]), cmap='hsv',vmin=0,vmax=1)
        self.x_data, self.y_data,self.color = [], [], []
        self.start=time.time()
        self.map_ = ConeOdom()
        self.yellow_cones = []
        self.blue_cones = []
        self.precision = 0.2
        self.waypoints_stamped = WaypointsStamped()
        self.load_ros_parameters()
        self.initialise_ros_subscriber()
        self.initialise_ros_publisher()

    def plot_init(self):
        self.ax.set_xlim(-70, 70)
        self.ax.set_ylim(-50, 100)
        return self.ln

    def update_plot(self,frame):
        end=time.time()
        if end-self.start >=1 and len(self.x_data)!=0 :           
            self.ln.set_offsets(np.c_[np.array(self.x_data),np.array(self.y_data)])
            self.ln.set_array(np.array(self.color))
            self.start=end
        return self.ln

    def get_headers(self):
        self.waypoints_stamped.header.frame_id = "map"
        self.waypoints_stamped.header.stamp = rospy.Time.now()
    
    def publishWaypoints(self):
        self.get_headers()
        self.waypointsPublisher_.publish(self.waypoints_stamped)

    def load_ros_parameters(self):
        self.slam_state_topic_name_ = \
            rospy.get_param('/Planning_node/slam_state_topic_name',
                            '/fsds/testing_only/odom')

        self.slam_map_topic_name_ = \
            rospy.get_param("/Planning_node/slam_map_topic_name",
                            "/ekf_slam_output")

        self.waypoints_topic_name_ = \
            rospy.get_param("/Planning_node/waypoints_topic_name",
                            "/mapping/waypoints")

    def initialise_ros_subscriber(self):
        rospy.loginfo("Subscibe to topics")
        rospy.Subscriber(self.slam_map_topic_name_, data_class=Track,
                         queue_size=1, callback=self.get_map_from_topic)
        # rospy.Subscriber(self.slam_map_topic_name_, data_class=ConeOdom,
                        #  queue_size=1, callback=self.get_map_from_topic)

    def get_map_from_topic(self, map_topic):
        # print(map_topic)
        t1 = time.time()
        self.map_ = map_topic
        self.map_to_cones()
        print("Time, ", time.time() - t1)

    def map_to_cones(self):
        self.blue_cones = []
        self.yellow_cones = []
        for points in self.map_.track:
            if points.color == 0:
                self.blue_cones.append(points)
            elif points.color == 1:
                self.yellow_cones.append(points)

        # print(self.blue_cones)
        # print(self.yellow_cones)
        self.generate_waypoints()

    def generate_waypoints(self):
        for yellow in self.yellow_cones:
            correspond_blue = self.find_closest_blue_cone(yellow)
            self.calculate_points(yellow, correspond_blue)
            self.x_data.append(yellow.location.x)
            self.y_data.append(yellow.location.y)
            self.color.append(0.16)
            self.x_data.append(correspond_blue.location.x)
            self.y_data.append(correspond_blue.location.y)
            self.color.append(0.66)
        self.calculate_dijkstra_path(self.waypoints_stamped.waypoints)
        self.waypoints_stamped.waypoints = self.generate_dense_points(
                                        self.waypoints_stamped.waypoints)

    def find_closest_blue_cone(self, yellow):
        closest_blue = min(self.blue_cones, key=lambda x: math.hypot(
            yellow.location.x - x.location.x,
            yellow.location.y - x.location.y))   
        return closest_blue

    def calculate_points(self, yellow, blue):
        pass

    def calculate_dijkstra_path(self, dijkstra_list):
        pass

    def generate_dense_points(self, points):
        dense_waypoints = []
        # for i in range(len(points)):
        #     new_p = Waypoint()
        #     new_p.position.x = points[i].position.x
        #     new_p.position.y = points[i].position.y
        #     new_p.position.z = points[i].position.z
        #     new_p.desired_velocity = 4.0
        #     self.x_data.append(new_p.position.x)
        #     self.y_data.append(new_p.position.y)
        #     self.color.append(0.5)
        #     dense_waypoints.append(new_p)

        for i in range(1, len(points)):
            dx = points[i].position.x - \
                points[i - 1].position.x
            dy = points[i].position.y - \
                points[i - 1].position.y
            d = math.hypot(dx, dy)

            nm_add_points = d / self.precision
            for j in range(0, int(nm_add_points)):
                new_p = Waypoint()
                new_p.position.x = points[i - 1].position.x + \
                    self.precision * j * dx / d
                new_p.position.y = points[i - 1].position.y + \
                    self.precision * j * dy / d
                new_p.position.z = points[i - 1].position.z
                new_p.desired_velocity = 4.0
                dense_waypoints.append(new_p)

                self.x_data.append(new_p.position.x)
                self.y_data.append(new_p.position.y)
                self.color.append(0.5)
        return dense_waypoints
        # return points

    def initialise_ros_publisher(self):
        rospy.loginfo("Publish to topics")
        self.waypointsPublisher_ = rospy.Publisher(
            self.waypoints_topic_name_, WaypointsStamped,
            queue_size=1, latch=True)
