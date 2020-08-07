#!/usr/bin/env python
import rospy 

from ros_pololu_servo.msg import MotorCommand
from drive_control.msg import DriveCommand
from std_msgs.msg import Float64

class drive:
    def __init__(self):
        self.motor_topic = '/pololu/command'
        self.front_topic = '/front_controller/command'
        self.back_topic = '/back_controller/command'
        self.drive_topic = '/drive/command'
        self.cmd_motor = MotorCommand()
        self.cmd_front = Float64()
        self.cmd_back = Float64()
         
        #create publisher passing it the vel_topic_name and msg_type
        self.motor_pub = rospy.Publisher(self.motor_topic, MotorCommand, queue_size = 1)
        self.front_pub = rospy.Publisher(self.front_topic, Float64, queue_size = 1)
        self.back_pub = rospy.Publisher(self.back_topic, Float64, queue_size = 1)

        #create subscriber
        self.drive_sub = rospy.Subscriber(self.drive_topic, DriveCommand, self.scan_callback)


    def scan_callback(self, data):
        self.motor_speed = data.speed
        self.cmd_front = data.front_steer_angle
        self.cmd_back = data.rear_steer_angle

        if self.motor_speed < -0.5235:
            self.motor_speed = -0.5235
        if self.motor_speed > 0.3499:
            self.motor_speed = 0.3499
        self.cmd_motor.joint_name = 'motor'
        self.cmd_motor.position = self.motor_speed

        self.motor_pub.publish(self.cmd_motor)
        self.front_pub.publish(self.cmd_front)
        self.back_pub.publish(self.cmd_back)
            
        rospy.sleep(0.1)

if __name__ == "__main__":
	rospy.init_node('drive_node')
	obj = drive()
	rospy.spin()
