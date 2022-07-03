#!/usr/bin/env python3

import rospy
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import time
from fsd_common_msgs.msg import ConeOdom, WaypointsStamped
from fs_msgs.msg import Track


fig, ax = plt.subplots()
ln = plt.scatter(x=np.array([]), y=np.array(
    []), s=3.0, c=np.array([]), cmap='hsv', vmin=0, vmax=1)
x_data, y_data, color = [], [], []
x_data1, y_data1, color1 = [], [], []
start = time.time()
xEst = []


def plot_init():
    global ax, ln
    ax.set_xlim(-40, 50)
    ax.set_ylim(-30, 170)
    # ax.set_xlim(-40, 60)
    # ax.set_ylim(-30, 100)
    return ln


def update_plot(frame):
    global start, ln, x_data, y_data, color

    end = time.time()
    if end-start >= 1 and len(xEst) != 0:
        color.extend(color1)
        y_data.extend(y_data1)
        x_data.extend(x_data1)
        ln.set_offsets(
            np.c_[np.array(x_data), np.array(y_data)])
        ln.set_array(np.array(color))
        start = end
    return ln


def plot_callback(data):

    global x_data, y_data, color, xEst

    # xEst = data.cone_detections
    xEst = data.track

    est_c_x = []
    est_c_y = []
    colors = []

    for i in range(len(xEst)):
        # est_c_x.append(xEst[i].position.x)
        # est_c_y.append(xEst[i].position.y)
        # if(xEst[i].color.data == 'b'):
        #     colors.append(0.62)
        # if(xEst[i].color.data == 'y'):
        #     colors.append(0.16)
        # if(xEst[i].color.data == 'o'):
        #     colors.append(0.52)
        
        est_c_x.append(xEst[i].location.x)
        est_c_y.append(xEst[i].location.y)
        if(xEst[i].color == 0):
            colors.append(0.62)
        if(xEst[i].color == 1):
            colors.append(0.16)

    x_data = est_c_x
    y_data = est_c_y
    color = colors

def path_callback(data):

    global x_data1, y_data1, color1

    xEsts = data.waypoints

    est_c_xs = []
    est_c_ys = []
    colorss = []

    for i in range(len(xEsts)):
        est_c_xs.append(xEsts[i].position.x)
        est_c_ys.append(xEsts[i].position.y)
        colorss.append(0.4)

    x_data1 = est_c_xs
    y_data1 = est_c_ys
    color1 = colorss

def main():

    rospy.init_node('plot_node')

    while not rospy.is_shutdown():
        # rospy.Subscriber('/ekf_slam_output', ConeOdom,
        #                  callback=plot_callback)
        rospy.Subscriber('/fsds/testing_only/track', Track,
                         callback=plot_callback)
        rospy.Subscriber('/mapping/waypoints', WaypointsStamped,
                         callback=path_callback)
        ani = FuncAnimation(fig, update_plot, init_func=plot_init)

        plt.show()
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
