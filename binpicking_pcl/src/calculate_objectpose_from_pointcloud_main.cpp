#include "ros/ros.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloud.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloudRequest.h"
#include "binpicking_msgs/CalculateObjectposeFromPointcloudResponse.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//Image message
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "calculate_objectpose_from_pointcloud.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>


ros::Publisher image_pub; //image message publisher
sensor_msgs::Image image; //cache the image message


bool CalculateObjectposeFromPointcloud(binpicking_msgs::CalculateObjectposeFromPointcloud::Request  &request,
         binpicking_msgs::CalculateObjectposeFromPointcloud::Response &response)
{

	ROS_INFO("CalculateObjectposeFromPointcloud start");
	geometry_msgs::TransformStamped static_transformStamped;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	ROS_INFO("add static transform to object");
	/* add static transform to object */
	static_transformStamped.header.stamp = ros::Time::now();
	static_transformStamped.header.frame_id = "camera_depth_optical_frame";
	static_transformStamped.child_frame_id = "object_to_grasp";
	static_transformStamped.transform.translation.x = 0;
	static_transformStamped.transform.translation.y = 0;
	static_transformStamped.transform.translation.z = 0.38;
	static_transformStamped.transform.rotation.x = 0;
	static_transformStamped.transform.rotation.y = 0;
	static_transformStamped.transform.rotation.z = 0;
	static_transformStamped.transform.rotation.w = 1.0;
	static_broadcaster.sendTransform(static_transformStamped);

	/* wait a while */
	ros::Duration(1.0).sleep();
	ROS_INFO("get object pose relative to world ");
	/* get object pose relative to world */
	try{
		geometry_msgs::TransformStamped transformStamped;
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.pose.orientation.x = 0;
		poseStamped.pose.orientation.y = 0;
		poseStamped.pose.orientation.z = 0;
		poseStamped.pose.orientation.w = 1.0;
		transformStamped = tfBuffer.lookupTransform("base_link", "object_to_grasp", ros::Time(0));
		tf2::doTransform(poseStamped, poseStamped, transformStamped); // robotPose is the PoseStamped I want to transform

		response.success = true;
		response.object_pose = poseStamped;
	}
	catch (tf2::TransformException &ex) {
		ROS_ERROR("Error lookupTransform.");
		response.success = false;
	}

	pcl::PCLPointCloud2* point_cloud2 = new pcl::PCLPointCloud2;
	pcl_conversions::toPCL(request.pointcloud, *point_cloud2);  // From ROS-PCL to PCL2

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*point_cloud2, *point_cloud); // From PCL2 to PCL


	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_Pointcloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	calculate_objectpose_from_pointcloud(point_cloud, colored_Pointcloud);

#if 0
	pcl::PCLPointCloud2* colored_Pointcloud2 = new pcl::PCLPointCloud2;
	pcl::toPCLPointCloud2(*colored_Pointcloud, *colored_Pointcloud2); // From PCL to PCL2

	sensor_msgs::PointCloud2Ptr output_cloud(new sensor_msgs::PointCloud2);// = new sensor_msgs::PointCloud2;
	pcl_conversions::fromPCL(*colored_Pointcloud2, *output_cloud);  // From PCL2 to ROS-PCL
#endif



	#if 0

	//  calculate_objectpose_from_pointcloud(request.pointcloud, image);

	try
	{
	pcl::toROSMsg (request.pointcloud, image);
	}
	catch (std::runtime_error)
	{
	    ROS_ERROR("runtime_error");
	}

	image_pub.publish (image); //publish our cloud image
	#endif
	ROS_INFO("CalculateObjectposeFromPointcloud exit");
	/* return object pose in response*/
	return true;
}


int main(int argc, char **argv)
{


	ros::init(argc, argv, "calculate_object_pose_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("calculate_object_pose", CalculateObjectposeFromPointcloud);

	image_pub = n.advertise<sensor_msgs::Image> ("cloud_image", 30);

	ROS_INFO("Ready to calculate pose.");
	ros::spin();

	return 0;
}
