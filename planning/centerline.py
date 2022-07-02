
import rospy
from fsd_common_msgs.msg import Waypoint, WaypointsStamped, ConeOdom
import math
import matplotlib.pyplot as plt
import time
import numpy as np
from fs_msgs.msg import Track

class center():
    def __init__(self):
        self.x_data, self.y_data,self.color = [], [], []
        self.map_ = ConeOdom()
        self.yellow_cones = []
        self.blue_cones = []
        self.precision = 0.2
        self.waypoints_stamped = WaypointsStamped()
        self.load_ros_parameters()
        self.initialise_ros_subscriber()
        self.initialise_ros_publisher()

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
        

    def map_to_cones(self):
        self.blue_cones = []
        self.yellow_cones = []
        for points in self.map_.track:
            if points.color == 0:
                self.blue_cones.append(points)
            elif points.color == 1:
                self.yellow_cones.append(points)

        self.generate_waypoints()

    def find_closest_blue_cone(self, yellow):
        closest_blue = min(self.blue_cones, key=lambda x: math.hypot(
            yellow.location.x - x.location.x,
            yellow.location.y - x.location.y))   
        return closest_blue

    def generate_waypoints(self):
        for yellow in self.yellow_cones:
            correspond_blue = self.find_closest_blue_cone(yellow)
            p = Waypoint()
            p.position.x = (yellow.location.x + correspond_blue.location.x - 6) / 2.0
            p.position.y = (yellow.location.y + correspond_blue.location.y) / 2.0
            p.position.z = 0.0
            self.waypoints_stamped.waypoints.append(p)
            
    def initialise_ros_publisher(self):
        rospy.loginfo("Publish to topics")
        self.waypointsPublisher_ = rospy.Publisher(
            self.waypoints_topic_name_, WaypointsStamped,
            queue_size=1, latch=True)

    def get_headers(self):
        self.waypoints_stamped.header.frame_id = "map"
        self.waypoints_stamped.header.stamp = rospy.Time.now()

    def get_map_from_topic(self, map_topic):
        # print(map_topic)
        t1 = time.time()
        self.map_ = map_topic
        self.map_to_cones()
        print("Time, ", time.time() - t1)

    def publishWaypoints(self):
        self.get_headers()
        self.waypointsPublisher_.publish(self.waypoints_stamped)
