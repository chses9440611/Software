#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <boost/thread/thread.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/console.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>  
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ctime>
#include <boost/thread/thread.hpp>
using namespace std;
std::string name;
ros::Publisher pub_cloud; 
ros::Publisher pub_cloud_test;                                                       
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_record(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
tf::TransformBroadcaster* br;
Eigen::Matrix4f matrix_record;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
int counter = 0;
bool first_Set_Viewer = true;

tf::Transform transformEigenToTF(Eigen::Matrix4f tm)
{
	tf::Vector3 origin;
	// transform to double type
  	origin.setValue(static_cast<double>(tm(0,3)),static_cast<double>(tm(1,3)),static_cast<double>(tm(2,3)));

  	tf::Matrix3x3 tf3d;
  	tf3d.setValue(static_cast<double>(tm(0,0)), static_cast<double>(tm(0,1)), static_cast<double>(tm(0,2)), 
        static_cast<double>(tm(1,0)), static_cast<double>(tm(1,1)), static_cast<double>(tm(1,2)), 
        static_cast<double>(tm(2,0)), static_cast<double>(tm(2,1)), static_cast<double>(tm(2,2)));

  	tf::Quaternion tfqt;
  	tf3d.getRotation(tfqt);

  	tf::Transform trans;
  	trans.setOrigin(origin);
  	trans.setRotation(tfqt);
	return trans;
}
tf::Transform icpWithPointXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target)
{
	clock_t begin = clock();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_1(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree_1->setInputCloud(cloud_source);
	tree_2->setInputCloud(cloud_target);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(cloud_source);
	icp.setInputTarget(cloud_target);
	icp.setMaxCorrespondenceDistance(1500); 
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations(500);
	icp.align(*cloud_icp);

  	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout<<"icp compute time = " << elapsed_secs<<endl;

	Eigen::Matrix4f trans_eigen = icp.getFinalTransformation() ;
	pcl::transformPointCloud(*cloud_source, *cloud_test, trans_eigen);
	matrix_record = matrix_record * trans_eigen;
	tf::Transform trans = transformEigenToTF(matrix_record);


	pub_cloud_test.publish(*cloud_test);
	pub_cloud.publish(*cloud_target);
	return trans;

}

void cbPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	pcl::fromROSMsg (*cloud_msg, *cloud);
	pcl::copyPointCloud(*cloud,*cloud_feature);
	if(counter == 6)
	{
		counter = 0;

		// normal of PCL -> feature
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud);
		ne.setSearchMethod(tree_xyz);
		ne.setRadiusSearch(0.2);
		ne.compute(*cloud_normals);
		/*for (int i = 0; i< cloud_normals->points.size(); i++)
		{
			cloud_normals->points[i].x = cloud_feature->points[i].x;
			cloud_normals->points[i].y = cloud_feature->points[i].y;
			cloud_normals->points[i].z = cloud_feature->points[i].z;
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
		sift.setInputCloud(cloud_feature);
		sift.setSearchMethod(tree_sift);
		sift.setScales(min_scale, n_octaves, n_scales_per_octave);
		sift.setMinimumContrast(min_contrast);
		sift.compute(*cloud_sift);
		*/

		//ICP
		if (cloud_feature_record->points.size() > 0 )
		{
			tf::Transform trans = icpWithPointXYZ(cloud_feature, cloud_feature_record);
 		  	try
		  	{
		  		br->sendTransform(tf::StampedTransform(trans, cloud_msg->header.stamp, "odom_frame", "base_link_3Dlidar"));
		  	}
		  	catch(tf::TransformException &ex)
		  	{
				ROS_ERROR("transfrom exception : %s",ex.what());
		  	}

		}
		//Visualization
		if (first_Set_Viewer)
		{
			first_Set_Viewer = false;
			viewer->addPointCloud<pcl::PointXYZ> (cloud_feature, "cloud");
			viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_feature, cloud_normals, 10, 0.5, "cloud_normals");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_normals");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_normals");

		}
		viewer->updatePointCloud(cloud_feature, "cloud");
		viewer->removePointCloud ("cloud_normals");
		viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_feature, cloud_normals, 10, 2, "cloud_normals");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_normals");
		pcl::copyPointCloud(*cloud_feature,*cloud_feature_record);

	}
	else
		counter++;
}

void cbViewer(const ros::TimerEvent& event)
{
	viewer->spinOnce();
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_odometry_node");
	name = ros::this_node::getName();
	ROS_INFO("[%s] Initializing ", name.c_str());

	ros::NodeHandle nh("~");

	tf::TransformBroadcaster br_instance;
	br = &br_instance;

	for (int x = 0;x<4;x++)
	{
		for (int y=0;y<4;y++)
		{
			if (x == y)
				matrix_record(x, y) = 1;
			else
				matrix_record(x, y) = 0;
		}
	}

	ros::Subscriber sub_velodyne_pcl = nh.subscribe("cloud_preprocess", 1000, cbPointCloud);
	pub_cloud = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("cloud_out", 1000);
	pub_cloud_test = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("cloud_out_test", 1000);

	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), cbViewer);
	
	ros::spin();

	return 0;
}
