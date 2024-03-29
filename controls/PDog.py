import rospy
from fs_msgs.msg import ControlCommand
from fsd_common_msgs.msg import WaypointsStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import numpy as np
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import time 

TARGET_SPEED = 3.5
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
         self.Kp = 1
         self.k = 1
         self.Kd = 15
         self.current_state = None
         self.x_pos = []
         self.y_pos = []
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
        self.x_pos1 = []
        self.y_pos1 = []
        for obj in self.waypoints_list:
            self.x_pos1.append(obj.position.x)
            self.y_pos1.append(obj.position.y)

        self.x_pos = np.array(self.x_pos1)
        self.y_pos = np.array(self.y_pos1)

    def slam_state_callback(self, state):
        self.current_state = state   

    def velocity_estimate_callback(self, velocity_):

        vx = velocity_.twist.linear.x
        vy = velocity_.twist.linear.y
        yaw = euler_from_quaternion([self.current_state.pose.pose.orientation.x, self.current_state.pose.pose.orientation.y, self.current_state.pose.pose.orientation.z, self.current_state.pose.pose.orientation.w])[2]
        self.velocity = abs((vx * np.cos(yaw) + vy * np.sin(yaw)))

    def initialisePublisher(self):
        rospy.loginfo('Publish to topics')
        self.control_command_publisher = rospy.Publisher('/fsds/control_command', ControlCommand, queue_size=1)

# LATERAL CONTROL

    def calc_nearestWaypoint(self):
        if self.x_pos != [] and self.y_pos != []:
            x_diff = abs(self.x_pos - self.currState.x)
            y_diff = abs(self.y_pos - self.currState.y)
            dist = np.sqrt( (x_diff ** 2) + (y_diff **2) )
            return np.argmin(dist)
        else:
            return 0
        
    def calc_crossTrackError(self):
        nearest_waypoint_index = self.calc_nearestWaypoint()
        nearestx_diff = (self.x_pos[nearest_waypoint_index] - self.currState.x)
        nearesty_diff = (self.y_pos[nearest_waypoint_index] - self.currState.y) 
        if nearestx_diff > 0:
            return np.hypot(nearestx_diff, nearesty_diff)
        else:
            return -1 * np.hypot(nearestx_diff, nearesty_diff)
    
    def calc_steeringAngle(self):
        ep = self.calc_crossTrackError()
        ed = (ep - self.previous_ep)
        steering_angle = (self.Kp * ep) + (self.Kd * ed)
        print(ep)
        # if ep < -4:
        #     ep = 1
        self.previous_ep = ep
        return steering_angle

    def publishControlCommands(self, steer, vel):
        control = ControlCommand()
        control.steering = steer
        control.throttle = vel
        self.control_command_publisher.publish(control)
    
    def convertPiToPi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

#LONGITUDINAL CONTROL

    def proportionalControl(self, target, current):
        a = self.k * (target - current)
        vel = 0
        if (a < 0):
            a = 0
        elif (a >= 1):
            a = 1
        else:
            a = (a / 24)
        return a

    def runPD(self):
        self.currState = State(self.current_state.pose.pose.position.x, self.current_state.pose.pose.position.y, self.convertPiToPi(euler_from_quaternion([self.current_state.pose.pose.orientation.x, self.current_state.pose.pose.orientation.y, self.current_state.pose.pose.orientation.z, self.current_state.pose.pose.orientation.w])[2]), self.velocity)
        self.steer = self.calc_steeringAngle()
        self.target_speed = TARGET_SPEED
        vel = self.proportionalControl(self.target_speed, self.velocity)
        self.publishControlCommands(self.steer, vel)
        self.plot(self.x_pos, self.y_pos)

    def plot(self, x_pos, y_pos):
        plt.cla()
        plt.plot(x_pos, y_pos, "ro", label="course")
        plt.plot(self.currState.x, self.currState.y, "bo", label="trajectory")
        plt.pause(0.001)
