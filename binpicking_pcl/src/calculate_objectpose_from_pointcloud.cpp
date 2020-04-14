#include "ros/ros.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloud.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloudRequest.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloudResponse.h"


bool CalculateObjectposeFromPointcloud(binpicking_msgs::CalculateObjectposeFromPointcloud::Request  &request,
         binpicking_msgs::CalculateObjectposeFromPointcloud::Response &response)
{
  /* add static transform to object */

  /* get object pose relative to world */

  /* return object pose in response*/
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "calculate_object_pose_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calculate_object_pose", CalculateObjectposeFromPointcloud);
  ROS_INFO("Ready to calculate pose.");
  ros::spin();

  return 0;
}
