import rospy
import time
from PD import PD

t1 = time.time()

def main():
    rospy.init_node('Control_node')
    controller = PD()
    while not rospy.is_shutdown():
        if controller.current_state_ != None and controller.x_pos != None and controller.velocity != None:
            t2 = time.time()
            if t2 - t1 == 5:
                controller.runPD(0)
            else:
                controller.runPD(2)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
