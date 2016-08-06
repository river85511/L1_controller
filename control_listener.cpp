#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */


void chatterCallback(const std_msgs::Float32MultiArray& msg)
{

  ROS_INFO("steering = %f\t u = %f",msg.data[0],msg.data[1]);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "control_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("l1control", 1000, chatterCallback);

  ros::spin();

  return 0;
}
