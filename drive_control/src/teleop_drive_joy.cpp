#include <drive_control/DriveCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "teleop_drive_joy.h"

#include <map>
#include <string>


namespace teleop_drive_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopDriveJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendDriveMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);

  ros::Subscriber joy_sub;
  ros::Publisher drive_pub;

  int enable_button;
  int enable_turbo_button;

  std::map<std::string, int> axis_map;
  std::map< std::string, std::map<std::string, double> > scale_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopDriveJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopDriveJoy::TeleopDriveJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->drive_pub = nh->advertise<drive_control::DriveCommand>("drive/command", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopDriveJoy::Impl::joyCallback, pimpl_);

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (nh_param->getParam("axis_control_value", pimpl_->axis_map["control_value"]))
  {
    nh_param->getParam("scale_control_value", pimpl_->scale_map["normal"]["control_value"]);
    nh_param->getParam("scale_control_value_turbo", pimpl_->scale_map["turbo"]["control_value"]);
  }
  else
  {
    nh_param->param<int>("axis_control_value", pimpl_->axis_map["control_value"], 1);
    nh_param->param<double>("scale_control_value", pimpl_->scale_map["normal"]["control_value"], 0.5);
    nh_param->param<double>("scale_control_value_turbo", pimpl_->scale_map["turbo"]["control_value"], 1.0);
  }

  if (nh_param->getParam("axis_steer", pimpl_->axis_map["steer"]))
  {
    nh_param->getParam("scale_steer", pimpl_->scale_map["normal"]["steer"]);
    nh_param->getParam("scale_steer_turbo", pimpl_->scale_map["turbo"]["steer"]);
  }
  else
  {
    nh_param->param<int>("axis_steer", pimpl_->axis_map["steer"], 1);
    nh_param->param<double>("scale_steer", pimpl_->scale_map["normal"]["steer"], 0.5);
    nh_param->param<double>("scale_steer_turbo", pimpl_->scale_map["turbo"]["steer"], 1.0);
  }

  ROS_INFO_NAMED("TeleopDriveJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopDriveJoy",
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_map.begin();
      it != pimpl_->axis_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopDriveJoy", "Axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopDriveJoy",
        "Turbo for axis %s is scale %f.", it->first.c_str(), pimpl_->scale_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopDriveJoy::Impl::sendDriveMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  drive_control::DriveCommand drive_msg;

  drive_msg.control_value = getVal(joy_msg, axis_map, scale_map[which_map], "control_value");
  drive_msg.front_steer_angle = getVal(joy_msg, axis_map, scale_map[which_map], "steer");
  drive_msg.rear_steer_angle = 0.0;

  drive_pub.publish(drive_msg);
  sent_disable_msg = false;
}

void TeleopDriveJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (enable_turbo_button >= 0 &&
      joy_msg->buttons.size() > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendDriveMsg(joy_msg, "turbo");
  }
  else if (joy_msg->buttons.size() > enable_button &&
           joy_msg->buttons[enable_button])
  {
    sendDriveMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default.
      drive_control::DriveCommand drive_msg;
      drive_pub.publish(drive_msg);
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_drive_joy
