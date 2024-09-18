#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from icecream import ic


class command_pub:
    def __init__(self):
        rospy.init_node('servo_commands', anonymous=True)
        self.wheel1 = rospy.Publisher('/wheel1_controller/command', Float64, queue_size=10)
        self.wheel2 = rospy.Publisher('/wheel2_controller/command', Float64, queue_size=10)
        self.wheel3 = rospy.Publisher('/wheel3_controller/command', Float64, queue_size=10)
        self.wheel4 = rospy.Publisher('/wheel4_controller/command', Float64, queue_size=10)
        self.joint1 = rospy.Publisher('/joint1_controller/command', Float64, queue_size=10)
        self.joint2 = rospy.Publisher('/joint2_controller/command', Float64, queue_size=10)
        self.joint3 = rospy.Publisher('/joint3_controller/command', Float64, queue_size=10)
        self.joint4 = rospy.Publisher('/joint4_controller/command', Float64, queue_size=10)

    def set_throttle_steer(self,Twist):
    
        throttle = Twist.linear.x*4.474
        ic(Twist.linear.x)
        ic(Twist.angular.z)
        steer = -(Twist.angular.z)

        self.wheel1.publish(throttle)
        self.wheel2.publish(throttle)
        self.wheel3.publish(throttle)
        self.wheel4.publish(throttle)

        self.joint1.publish(steer)
        self.joint2.publish(steer)
        self.joint3.publish(0)
        self.joint4.publish(0)


    def servo_commands(self):
        rospy.Subscriber("/cmd_vel", Twist, self.set_throttle_steer,queue_size=1)

if __name__ == '__main__':
   
    command_pub = command_pub()   
    try:
        command_pub.servo_commands()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
