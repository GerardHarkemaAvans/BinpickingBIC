#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include "calculate_objectpose_from_pointcloud.h"

//void calculate_objectpose_from_pointcloud(const sensor_msgs::PointCloud2ConstPtr& input){
void calculate_objectpose_from_pointcloud(sensor_msgs::PointCloud2 input){

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (input, cloud);


#if 0
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }
#endif
}
