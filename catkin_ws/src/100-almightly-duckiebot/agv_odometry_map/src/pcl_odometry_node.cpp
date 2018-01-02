#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
using namespace std;
std::string name;
ros::Publisher pub_cloud;                                                       

void cbPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
	/*
	// normal of PCL
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setInputCloud(cloud_outlier);
	ne.setSearchMethod(tree_xyz);
	ne.setRadiusSearch(0.2);
	ne.compute(*cloud_normals);
	for(int i = 0; i<cloud_normals->points.size(); ++i)
	{
		cloud_normals->points[i].x = cloud_voxel->points[i].x;
		cloud_normals->points[i].y = cloud_voxel->points[i].y;
		cloud_normals->points[i].z = cloud_voxel->points[i].z;
	}*/
	
	/*
	// Parameters for sift computation
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree_sift(new pcl::search::KdTree<pcl::PointNormal>);
	const float min_scale = 0.01f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.001f;
	//Sift 
	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointXYZ> sift;
	sift.setInputCloud(cloud_normals);
	sift.setSearchMethod(tree_sift);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.compute(*cloud_sift);
	*/

	pub_cloud.publish(*cloud_msg);

}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_odometry_node");
	name = ros::this_node::getName();
	ROS_INFO("[%s] Initializing ", name.c_str());

	ros::NodeHandle nh("~");
	ros::Subscriber sub_velodyne_pcl = nh.subscribe("cloud_preprocess", 1000, cbPointCloud);

	pub_cloud = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("cloud_out", 1000);

	ros::spin();

	return 0;
}
