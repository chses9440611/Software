#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
using namespace std;
std::string name;
ros::Publisher pub_cloud_voxel;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

void cbPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	pcl::PCLPointCloud2::Ptr cloud(new  pcl::PCLPointCloud2), cloud_blob(new pcl::PCLPointCloud2);
	PointCloudXYZ::Ptr cloud_filter(new PointCloudXYZ);
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Voxel Filter to DownSampling
	pcl::VoxelGrid< pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize (0.1f, 0.1f, 0.1f);
	sor.filter(*cloud_blob);

	// Convert to the templated PointCloudXYZ
	pcl:fromPCLPointCloud2(*cloud_blob, *cloud_filter);

	//sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCL(*cloud_filtered, output);
	pub_cloud_voxel.publish(*cloud_filter);

}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_odometry_node");
	name = ros::this_node::getName();
	ROS_INFO("[%s] Initializing ", name.c_str());

	ros::NodeHandle nh("~");
	ros::Subscriber sub_velodyne_pcl = nh.subscribe("/velodyne_points", 1000, cbPointCloud);

	pub_cloud_voxel = nh.advertise<PointCloudXYZ>("cloud_voxel", 1000);

	ros::spin();

	return 0;
}
