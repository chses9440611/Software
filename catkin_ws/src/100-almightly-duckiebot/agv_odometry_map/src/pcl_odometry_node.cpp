#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <boost/thread/thread.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>  
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/boost_graph.h>
#include <pcl/registration/boost.h>
#include <pcl/registration/elch.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <grid_map_msgs/GridMap.h>
using namespace std;
struct Point
{
	float x;
	float y;
	float th;
	ros::Time time;
};
std::string name;
ros::Publisher pub_cloud_target; 
ros::Publisher pub_cloud_test;   
ros::Publisher pub_odom; 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_record(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZ>);
tf::TransformBroadcaster* br;
tf::TransformListener *listener;
Eigen::Matrix4f matrix_record;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
pcl::visualization::PCLPlotter plotter;
int counter = 0;
bool first_Set_Viewer = true;
Point wheel_state, wheel_state_record;
tf::Transform wheel_odom_trans, wheel_odom_record_trans;
void drawOdometry(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	nav_msgs::Odometry odom;
	odom.header.stamp = cloud_msg->header.stamp;
	odom.header.frame_id = "odom_frame";
	odom.child_frame_id = "base_link_3Dlidar";
	tf::StampedTransform trans;
	try{
		listener->waitForTransform("odom_frame", "base_link_3Dlidar",  cloud_msg->header.stamp, ros::Duration(0.5));
		listener->lookupTransform("odom_frame", "base_link_3Dlidar",  cloud_msg->header.stamp, trans);
		tf::Vector3 origin = trans.getOrigin();
		tf::Quaternion rotate = trans.getRotation();
		wheel_state.x = origin.getX ();
		wheel_state.y = origin.getY (); 
		wheel_state.th = tf::getYaw(rotate);
		wheel_state.time = cloud_msg->header.stamp;


	}
  	catch(tf::TransformException e)
  	{
		ROS_INFO("[%s] LookupTransform odom_frame to base_link_3Dlidar timed out ", name.c_str());
  	}

	odom.pose.pose.position.x = wheel_state_record.x;
	odom.pose.pose.position.y = wheel_state_record.y;
	odom.pose.pose.position.z = 0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(wheel_state_record.th);
	float dt = wheel_state.time.toSec() - wheel_state_record.time.toSec();
	if (dt != 0)	
	{
		odom.twist.twist.linear.x = (wheel_state.x - wheel_state_record.x) * 1/dt ;
		odom.twist.twist.linear.y = (wheel_state.y - wheel_state_record.y) * 1/dt ;
		odom.twist.twist.angular.z = (wheel_state.th - wheel_state_record.th) * 1/dt ;
		pub_odom.publish(odom);
	}
	wheel_state_record = wheel_state;

}
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
Eigen::Matrix4f transformTFtoEigen(tf::Transform trans)
{
	Eigen::Matrix4f tm;
	Eigen::Affine3d tmp;
	tf::transformTFToEigen(trans, tmp);
	tf::Vector3 origin = trans.getOrigin();
	tf::Quaternion rotate = trans.getRotation();
	tm = tmp.matrix().cast<float>();
	tm(0 ,3) = origin.getX();
	tm(1, 3) = origin.getY();
	tm(2, 3) = 0;
	tm(3, 3) = 1;
	tm(3, 0) = tm(3, 1) = tm(3, 2) = 0;
	//cout << tm <<endl;
	return tm;
}
void setIdentity(Eigen::Matrix4f& trans_eigen)
{
	for (int x = 0;x<4;x++)
	{
		for (int y=0;y<4;y++)
		{
			if (x == y)
				trans_eigen(x, y) = 1;
			else
				trans_eigen(x, y) = 0;
		}
	}

}
tf::Transform ndtWithPointXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target, Eigen::Matrix4f initial)
{
	clock_t begin = clock();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon (0.01);
	ndt.setStepSize (0.1);
	ndt.setResolution (1.0);
	ndt.setMaximumIterations (30);
	ndt.setInputSource (cloud_source);
	ndt.setInputTarget (cloud_target);
	ndt.align (*cloud_ndt, initial);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout<<"initial = "<<initial<<endl;
	cout<<"ndtWithPointXYZ compute time = " << elapsed_secs<<", score =  "<<ndt.getFitnessScore() <<endl;
	Eigen::Matrix4f trans_eigen = ndt.getFinalTransformation() ;
	if (ndt.getFitnessScore() > 0.7 || trans_eigen(0, 3) > 0.45){
		//setIdentity(trans_eigen);
		trans_eigen = initial;
	}

	matrix_record = matrix_record * trans_eigen;
	tf::Transform trans = transformEigenToTF(matrix_record);
	
	return trans;
}
tf::Transform icpWithNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target, Eigen::Matrix4f initial)
{
	clock_t begin = clock();

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_test_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_source_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_target_normals(new pcl::PointCloud<pcl::PointXYZINormal>);

	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);

	// normal of PCL and combine to pointxyz
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_source;
	ne_source.setInputCloud(cloud_source);
	ne_source.setSearchMethod(tree_xyz);
	ne_source.setRadiusSearch(0.6);
	ne_source.compute(*source_normals);
	pcl::concatenateFields( *cloud_source, *source_normals, *cloud_with_source_normals );

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_target;
	ne_source.setInputCloud(cloud_target);
	ne_source.setSearchMethod(tree_xyz);
	ne_source.setRadiusSearch(0.6);
	ne_source.compute(*target_normals);
	pcl::concatenateFields( *cloud_target, *target_normals, *cloud_with_target_normals );

	//ICP
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree_1(new pcl::search::KdTree<pcl::PointXYZINormal>);
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZINormal>);
	tree_1->setInputCloud(cloud_with_source_normals);
	tree_2->setInputCloud(cloud_with_target_normals);
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;
	icp.setInputCloud(cloud_with_source_normals);
	icp.setInputTarget(cloud_with_target_normals);
	icp.setSearchMethodSource(tree_1);
	icp.setSearchMethodTarget(tree_2);
	icp.setMaxCorrespondenceDistance(500); 
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(0.1);
	icp.setMaximumIterations(50);
	icp.setRANSACOutlierRejectionThreshold(0.1);
	icp.align(*cloud_icp);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout<<"icpWithNormal compute time = " << elapsed_secs<<", score =  "<<icp.getFitnessScore() <<endl;
	Eigen::Matrix4f trans_eigen = icp.getFinalTransformation() ;
	if (icp.getFitnessScore() > 0.7 || trans_eigen(0, 3) > 0.45){
		//setIdentity(trans_eigen);
		trans_eigen = initial;
	}
	
	cout << trans_eigen << endl;
	pcl::transformPointCloud(*cloud_with_source_normals, *cloud_test_with_normals, trans_eigen);
	matrix_record = matrix_record * trans_eigen;
	tf::Transform trans = transformEigenToTF(matrix_record);

	pub_cloud_test.publish(*cloud_test_with_normals);
	pub_cloud_target.publish(*cloud_with_target_normals);
	if (first_Set_Viewer)
	{
		first_Set_Viewer = false;
		viewer->addPointCloud<pcl::PointXYZ> (cloud_source, "cloud");
		viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_source, source_normals, 10, 0.5, "cloud_normals");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_normals");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_normals");

	}
	viewer->updatePointCloud(cloud_source, "cloud");
	viewer->removePointCloud ("cloud_normals");
	viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_source, source_normals, 10, 2, "cloud_normals");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_normals");
	return trans;

}

tf::Transform icpWithPointXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target, Eigen::Matrix4f initial)
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
	icp.setSearchMethodSource(tree_1);
	icp.setSearchMethodTarget(tree_2);
	icp.setMaxCorrespondenceDistance(1500); 
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.1);
	icp.setMaximumIterations(600);
	icp.setRANSACOutlierRejectionThreshold(0.01);
	icp.align(*cloud_icp);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout<<"icpWithNormal compute time = " << elapsed_secs<<", score =  "<<icp.getFitnessScore() <<endl;
	Eigen::Matrix4f trans_eigen = icp.getFinalTransformation() ;
	if (icp.getFitnessScore() > 0.7 || trans_eigen(0, 3) > 0.45){
		//setIdentity(trans_eigen);
		trans_eigen = initial;
	}
	cout<<"icpWithPointXYZ compute time = " << elapsed_secs<<", score =  "<<icp.getFitnessScore() <<endl;
	//cout << trans_eigen << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_test, trans_eigen);
	matrix_record = matrix_record * trans_eigen;
	tf::Transform trans = transformEigenToTF(matrix_record);


	pub_cloud_test.publish(*cloud_test);
	pub_cloud_target.publish(*cloud_target);
	return trans;

}

void cbPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg (*cloud_msg, *cloud);
	pcl::copyPointCloud(*cloud,*cloud_feature);
	if(counter == 5)
	{
		counter = 0;
		//ICP
		tf::StampedTransform listen_trans;
		try{	
			listener->waitForTransform("odom_frame", "base_link_wheel",  cloud_msg->header.stamp, ros::Duration(0.5));
			listener->lookupTransform("odom_frame", "base_link_wheel",  cloud_msg->header.stamp, listen_trans);
			wheel_odom_trans = listen_trans;
			if (cloud_feature_record->points.size() > 0 )
			{
				tf::Transform initial = wheel_odom_record_trans.inverseTimes(wheel_odom_trans);
				tf::Transform trans = ndtWithPointXYZ(cloud_feature, cloud_feature_record, transformTFtoEigen(initial));
				try{	
					br->sendTransform(tf::StampedTransform(trans, cloud_msg->header.stamp, "odom_frame", "base_link_3Dlidar"));
				}
				catch(tf::TransformException &ex){
					ROS_ERROR("transfrom exception : %s",ex.what());
				}			
				drawOdometry(cloud_msg);
			}
		}
		catch(tf::TransformException &ex){
			ROS_INFO("[%s] LookupTransform odom_frame to base_link_wheel timed out ", name.c_str());
		}

		wheel_odom_record_trans = wheel_odom_trans;
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
	tf::TransformListener listener_instance;
	listener = &listener_instance;
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
	wheel_state.x = 0;
	wheel_state.y = 0;
	wheel_state.th = 0;
	wheel_state_record = wheel_state;
	//ros::Subscriber sub_velodyne_pcl = nh.subscribe("/velodyne_points", 1000, cbPointCloud);
	ros::Subscriber sub_velodyne_pcl = nh.subscribe("cloud_preprocess", 1000, cbPointCloud);
	pub_cloud_target= nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("cloud_out_target", 1000);
	pub_cloud_test = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("cloud_out_test", 1000);
	pub_odom = nh.advertise<nav_msgs::Odometry>("velodyne_odometry", 1000);	
	//Visualization
	
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	ros::Timer timer = nh.createTimer(ros::Duration(0.1), cbViewer);
	
	ros::spin();

	return 0;
}
