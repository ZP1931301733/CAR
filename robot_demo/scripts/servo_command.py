#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
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

    def set_throttle_steer(self,data):
    
        throttle = data.drive.speed*4.474
        steer = data.drive.steering_angle

        self.wheel1.publish(throttle)
        self.wheel2.publish(throttle)
        self.wheel3.publish(throttle)
        self.wheel4.publish(throttle)

        self.joint1.publish(steer)
        self.joint2.publish(steer)
        self.joint3.publish(steer)
        self.joint4.publish(steer)


    def servo_commands(self):
        rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, self.set_throttle_steer)

if __name__ == '__main__':
   
    command_pub = command_pub()
    try:
        command_pub.servo_commands()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
