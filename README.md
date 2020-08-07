Package to control dynamixels and pololu motor controller.

Run with `roslaunch drive_control drive_control.launch`.

Publish to `rostopic pub /drive/command drive_control/DriveCommand "speed: 0.0
front_steer_angle: 0.0
rear_steer_angle: 0.0" `

Speed: negative is backwards, zero is stop, positive is forwards. Values between [-0.5235,0.35].

Front_steer_angle: negative is left, postive is right. Values between [-0.25,0.25] in radians.

Rear_steer_angle: negative is right, postive is left. Values between [-0.25,0.25] in radians.

Note: steer angle right and left are in relation to the car moving forward.