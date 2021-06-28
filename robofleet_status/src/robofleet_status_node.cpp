/**
 * Simple ROS node to publish a basic status message to the Robofleet Webviz
 * 
 * The basic robot status message is made up of the following elements:
 * - string status {"online", "offline"}
 * - boolean is_ok
 * - float32 battery_level from 0-1
 * - string location "[BuildingName][Floor]: [coordinates]" e.g. "BLD2: 0, 10, 0"
 * 
 * Authors: Corrie Van Sice, 2021 and adapted from code by Kavan Sikand
 **/

#include <ctime>
#include <ros/ros.h>
#include "amrl_msgs/RobofleetStatus.h"
// TODO include your robot's relevant message types
#include "example_robot/RobotPowerData.h"


amrl_msgs::RobofleetStatus status_msg;
int timeout = 3;
time_t last_status_msg;

// TODO change the message type to match your robot data messages
void StatusCallback(const example_robot::RobotPowerData& msg) {
  // TODO pull out the relevant parts of the robot message to complete the RobofleetStatus message
  status_msg.battery_level = msg.battery_percent_full * 0.01;
  // TODO check other robot messages to get location, ok, etc.
  status_msg.status = "online";
  status_msg.is_ok = true;
  status_msg.location = "BLD2: 0, 0, 0";

  // log info and record time message was received
  ROS_INFO("Battery level is: %f", status_msg.battery_level);
  last_status_msg = time(NULL);
}

int main(int argc, char** argv)
{
  // initialize the node
  ros::init(argc, argv, "robofleet_status");
  ros::NodeHandle n;
  // TODO subscribe to the robot data topic(s)
  ros::Subscriber power_sub = n.subscribe("example_power_data_topic", 1, &StatusCallback);
  // create a publisher for outgoing robofleet status messages
  ros::Publisher status_pub = n.advertise<amrl_msgs::RobofleetStatus>("status", 1);

  double curr_time;
  ros::Rate loop_rate(10);

  ROS_INFO("Preparing messages for Robofleet WebViz");
 
  while (ros::ok()) {
    time_t curr_time = time(NULL);
  
    // haven't heard from the robot in too long
    if (curr_time - last_status_msg > timeout) {
      status_msg.status = "unknown";
      status_msg.is_ok = false;
      status_msg.battery_level = 0.0;
      status_msg.location = "";
      ROS_INFO("Stopped receiving robot data; timeout met");
    }
    // publish the status message
    status_pub.publish(status_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}