from re import search
from socket import timeout
import rospy
from fs_msgs.msg import ControlCommand
from fsd_common_msgs.msg import WaypointsStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import numpy as np
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.WB = 1.5

class PD():

    def __init__(self):
         self.initializeSubscribers()
         self.initialisePublisher()
         self.waypoints_list = []
         self.Kp = 2
         self.Kd = 2
         self.x_pos = None
         self.velocity = None
         self.steering_angle = 0
         self.previous_ep = 0

    def initializeSubscribers(self):
        rospy.loginfo('Subscribe to topics')
        rospy.Subscriber('/fsds/testing_only/odom', Odometry, queue_size=1, callback=self.slam_state_callback)
        rospy.Subscriber('/fsds/gss', TwistStamped, queue_size=1, callback=self.velocity_estimate_callback)
        rospy.Subscriber('/mapping/waypoints', WaypointsStamped, queue_size=1, callback=self.update_waypoints)

    def update_waypoints(self, waypoints_stamped):
        self.waypoints_list = waypoints_stamped.waypoints
        self.x_pos = []
        self.y_pos = []
        for obj in self.waypoints_list:
            self.x_pos.append(obj.position.x)
            self.y_pos.append(obj.position.y)

    def slam_state_callback(self, state):
        self.current_state_ = state   

    def velocity_estimate_callback(self, velocity_):
        vx = velocity_.twist.linear.x
        vy = velocity_.twist.linear.y
        yaw = euler_from_quaternion([self.current_state_.pose.pose.orientation.x, self.current_state_.pose.pose.orientation.y, self.current_state_.pose.pose.orientation.z, self.current_state_.pose.pose.orientation.w])[2]
        self.velocity = abs((vx * np.cos(yaw) + vy * np.sin(yaw)))

    def initialisePublisher(self):
        rospy.loginfo('Publish to topics')
        self.control_command_publisher = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)

    def calc_nearestWaypoint(self):
        x_diff = abs(self.x_pos - self.x)
        y_diff = abs(self.y_pos - self.y)
        dist = np.sqrt( (x_diff ** 2) + (y_diff **2) )
        return np.argmin(dist)
        
    def calc_crossTrackError(self):
        nearest_waypoint_index = self.calc_nearestWaypoint()
        nearestx_diff = abs(self.x_pos[nearest_waypoint_index] - self.x)
        nearesty_diff = abs(self.y_pos[nearest_waypoint_index] - self.y)
        return np.hypot(nearestx_diff, nearesty_diff)
    
    def calc_steeringAngle(self):
        ep = self.calc_crossTrackError()
        ed = (ep - self.previous_ep)
        steering_angle = (self.Kp * ep) + (self.Kd * ed) 
        self.previous_ep = ep
        return steering_angle

    def publishControlCommands(self, steer, vel):
        control = ControlCommand()
        control.steering = steer()
        control.throttle = vel
        self.control_command_publisher.publish(control)
    
    def convertPiToPi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def runPD(self):
        self.currState = State(self.current_state_.pose.pose.position.x, self.current_state_.pose.pose.position.y, self.convertPiToPi(euler_from_quaternion([self.current_state_.pose.pose.orientation.x, self.current_state_.pose.pose.orientation.y, self.current_state_.pose.pose.orientation.z, self.current_state_.pose.pose.orientation.w])[2]), self.velocity)
        vel = 5
        self.steer = self.calc_steeringAngle()
        self.publishControlCommands(self.steer, vel)

        





    
