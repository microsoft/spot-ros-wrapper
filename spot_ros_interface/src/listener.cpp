#include "ros/ros.h"
// #include "std_msgs/String.h"
#include "std_msgs/Float32.h"


void spotStateCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("spot_state", 1000, spotStateCallback);

  ros::spin();

  return 0;
}