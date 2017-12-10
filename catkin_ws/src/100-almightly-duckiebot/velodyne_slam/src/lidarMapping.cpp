#include <ros/ros.h>
//PCL lib
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/point_cloud.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
//Publisher
ros::Publisher pubLidarCloud;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	PointCloudXYZ::Ptr inputCloud(new PointCloudXYZ);
	PointCloudXYZ::Ptr cloud_filtered(new PointCloudXYZ);

	//Convert to PCL data type
	pcl::fromROSMsg(*cloud_msg, *inputCloud);

	//downsample filter
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(inputCloud);
	sor.setLeafSize(0.01, 0.01, 0.01);
	sor.filter(*cloud_filtered);

	//convert to ROS PCL
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_filtered, output);

	pubLidarCloud.publish(output);

}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "lidarMapping");
	ros::NodeHandle nh;

	//Subscriber
	ros::Subscriber subLidarCloud = nh.subscribe("/lidarMapping/velodyne_points", 10, cloud_cb);

	pubLidarCloud = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);


	//enter a loop, pumping callbacks
	ros::spin();
}