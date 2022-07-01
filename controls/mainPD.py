import rospy
from PD import PD


def main():
    rospy.init_node('Control_node')
    controller = PD()
    while not rospy.is_shutdown():
        print("Juniors kuch kaam nahi kar rahe")
        if controller.current_state != None and controller.velocity != None:
            # print("Agar yeh run hua, sablog ko mai nikal raha hai")
            controller.runPD()
        else:
            print(controller.current_state)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
