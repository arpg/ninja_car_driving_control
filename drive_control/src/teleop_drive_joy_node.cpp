#include "ros/ros.h"
#include "teleop_drive_joy.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "teleop_drive_joy_node");

  ros::NodeHandle nh(""), nh_param("~");
  teleop_drive_joy::TeleopDriveJoy joy_teleop(&nh, &nh_param);

  ros::spin();
}
