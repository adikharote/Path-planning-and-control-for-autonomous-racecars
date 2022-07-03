#! /usr/bin/env python3

import rospy
from centerLine import CenterLine
from voronoi import Voronoi
from dijkstra import Dijkstra
import matplotlib.pyplot as plt
import numpy as np


def main():
    rospy.init_node('Planning_node')

    algorithm = rospy.get_param(
        "/Planning_node/track_algorithm", "center-line")

    if algorithm == "center-line":
        controller = CenterLine()
    elif algorithm == "Voronoi":
        controller = Voronoi()
    elif algorithm == "Dijkstra":
        controller = Dijkstra()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        controller.run()
        # plt.scatter(x=np.array(controller.x_data), y=np.array(controller.y_data),s = 1.0, c=np.array(controller.color), cmap='hsv',vmin=0,vmax=1)
        # plt.ion()
        # plt.show()
        # plt.draw()
        # plt.pause(0.001)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
