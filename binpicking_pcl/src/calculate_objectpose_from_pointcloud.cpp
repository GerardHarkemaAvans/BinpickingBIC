#include "ros/ros.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloud.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloudRequest.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloudResponse.h"


bool CalculateObjectposeFromPointcloud(binpicking_msgs::CalculateObjectposeFromPointcloud::Request  &req,
         binpicking_msgs::CalculateObjectposeFromPointcloud::Response &res)
{
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", CalculateObjectposeFromPointcloud);
  ROS_INFO("Ready to calculate pose.");
  ros::spin();

  return 0;
}
