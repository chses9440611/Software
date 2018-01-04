#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
using namespace std;
std::string name;
ros::Publisher pub_cloud;                                                       
int counter = 0;
void cbPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	if(counter != 3)
	{
		counter++;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier(new pcl::PointCloud<pcl::PointXYZ>);
		

		// for ros publish PCL
		pcl::fromROSMsg (*cloud_msg, *cloud);

		// Voxel Filter to DownSampling
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setInputCloud(cloud);
		vg.setLeafSize (0.05f, 0.05f, 0.05f);
		vg.filter(*cloud_voxel);
		float max_range = 10, dis;
		for (int i=0 ; i <  cloud_voxel->points.size() ; i++)
		{
			dis = cloud_voxel->points[i].z * cloud_voxel->points[i].z + 
				cloud_voxel->points[i].x * cloud_voxel->points[i].x +
				cloud_voxel->points[i].y * cloud_voxel->points[i].y;
			dis = sqrt(dis);

			if (cloud_voxel->points[i].z < 0.05 || dis > max_range)
			{
				cloud_voxel->points.erase(cloud_voxel->points.begin()+i);
				cloud_voxel->width -=1 ;
				cloud_voxel->points.resize(cloud_voxel->width);
				i--;
			}
		}

		// filter outlier 
		/*
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud_voxel);
		sor.setMeanK(150);
		sor.setStddevMulThresh(0.8); 
		sor.filter(*cloud_outlier); */

		pub_cloud.publish(*cloud_voxel);
	}
	else
	{
		counter = 0;
	}

}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_preprocess_node");
	name = ros::this_node::getName();
	ROS_INFO("[%s] Initializing ", name.c_str());
	counter = 0;
	ros::NodeHandle nh("~");
	ros::Subscriber sub_velodyne_pcl = nh.subscribe("/velodyne_points", 1000, cbPointCloud);

	pub_cloud = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("cloud_preprocess", 1000);

	ros::spin();

	return 0;
}
