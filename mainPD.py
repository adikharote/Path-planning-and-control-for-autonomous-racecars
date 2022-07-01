import rospy
from PID import PD


def main():
    rospy.init_node('Control_node')
    controller = PD()
    while not rospy.is_shutdown():
        if controller.current_state_ != None and controller.x_pos != None and controller.velocity != None:
            controller.runPD()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
