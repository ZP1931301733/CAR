#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from robot_demo.msg import control
from icecream import ic


class command_pub:
    def __init__(self):
        rospy.init_node('servo_commands', anonymous=True)
        self.wheel1 = rospy.Publisher('/wheel1_controller/command', Float64, queue_size=1)
        self.wheel2 = rospy.Publisher('/wheel2_controller/command', Float64, queue_size=1)
        self.wheel3 = rospy.Publisher('/wheel3_controller/command', Float64, queue_size=1)
        self.wheel4 = rospy.Publisher('/wheel4_controller/command', Float64, queue_size=1)
        self.joint1 = rospy.Publisher('/joint1_controller/command', Float64, queue_size=1)
        self.joint2 = rospy.Publisher('/joint2_controller/command', Float64, queue_size=1)
        self.joint3 = rospy.Publisher('/joint3_controller/command', Float64, queue_size=1)
        self.joint4 = rospy.Publisher('/joint4_controller/command', Float64, queue_size=1)

    def set_throttle_steer(self,control):
    
        throttle1 = control.wheel1_speed*4.474
        throttle2 = control.wheel2_speed*4.474
        throttle3 = control.wheel3_speed*4.474
        throttle4 = control.wheel4_speed*4.474
        ic(control.wheel1_speed)
        ic(control.wheel2_speed)

        steer1 = control.joint1_angle
        steer2 = control.joint2_angle
        steer3 = control.joint3_angle
        steer4 = control.joint4_angle

        ic(1)
        self.wheel1.publish(throttle1)
        self.wheel2.publish(throttle2)
        self.wheel3.publish(throttle3)
        self.wheel4.publish(throttle4)

        self.joint1.publish(steer1)
        self.joint2.publish(steer2)
        self.joint3.publish(steer3)
        self.joint4.publish(steer4)


    def servo_commands(self):
        rospy.Subscriber("/control_comand", control, self.set_throttle_steer)

if __name__ == '__main__':
   
    command_pub = command_pub()   
    try:
        command_pub.servo_commands()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
