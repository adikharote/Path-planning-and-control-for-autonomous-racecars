import rospy
from centerline import center

def main():
    rospy.init_node('Planning_node')
    controller = center()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        controller.publishWaypoints()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
