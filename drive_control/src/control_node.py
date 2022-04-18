#!/usr/bin/env python
import rospy 

from drive_control.msg import DriveCommand
from std_msgs.msg import Float64, Int16

class drive:
    def __init__(self):
        self.drive_topic = 'drive/command'
        self.drive_sub = rospy.Subscriber(self.drive_topic, DriveCommand, self.drive_cmd_callback)

        self.control_mode = rospy.get_param("control_value_mode", 0) # default speed
        self.control_value_scale = rospy.get_param("control_value_scale", 1.0)
        self.control_value = Float64()
        self.speed_pub = rospy.Publisher('commands/motor/speed', Float64, queue_size = 1)
        self.accel_pub = rospy.Publisher('commands/motor/current', Float64, queue_size = 1)
        
        self.front_steer_scale = rospy.get_param("front_steer_scale", 1.0)
        self.front_steer = Float64()
        self.front_steer_pub = rospy.Publisher('front_controller/command', Float64, queue_size = 1)
        
        self.rear_steer_scale = rospy.get_param("rear_steer_scale", 1.0)
        self.rear_steer = Float64()
        self.rear_steer_pub = rospy.Publisher('back_controller/command', Float64, queue_size = 1)
        

    def drive_cmd_callback(self, data):
        self.control_value = data.control_value*self.control_value_scale
        self.front_steer = data.front_steer_angle*self.front_steer_scale
        self.rear_steer = data.rear_steer_angle*self.rear_steer_scale

        if self.control_mode==DriveCommand.CONTROL_MODE_SPEED:
            self.speed_pub.publish(self.control_value)
        elif self.control_mode==DriveCommand.CONTROL_MODE_ACCEL:
            self.accel_pub.publish(self.control_value)
        else:
            self.speed_pub.publish(self.control_value)

        self.front_steer_pub.publish(self.front_steer)
        self.rear_steer_pub.publish(self.rear_steer)

if __name__ == "__main__":
	rospy.init_node('drive_node')
	obj = drive()
	rospy.spin()
