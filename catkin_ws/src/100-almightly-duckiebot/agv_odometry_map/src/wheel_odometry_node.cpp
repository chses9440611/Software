#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "duckietown_msgs/WheelsCmdStamped.h"
#include <sensor_msgs/LaserScan.h>
using namespace std;
#define WIDTH 0.5 // (m)
#define WHEEL_RADIUS 0.085
struct Point
{
	float x;
	float y;
	float th;
	ros::Time time;
};

Point wheel_pos, wheel_pos_record;
std::string bot_name;
std::string node_name;
ros::Publisher pub_odom;
tf::TransformBroadcaster* br;

void drawWheelOdom()
{
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "map";
	odom.child_frame_id = "odom_frame";

	odom.pose.pose.position.x = wheel_pos.x;
  	odom.pose.pose.position.y = wheel_pos.y;
  	odom.pose.pose.position.z = 0.0;
  	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(wheel_pos.th);

  	//cal speed 
  	float dt = wheel_pos.time.toSec() - wheel_pos_record.time.toSec();
    odom.twist.twist.linear.x = (wheel_pos_record.x - wheel_pos.x) * 1/dt;
    odom.twist.twist.linear.y = (wheel_pos_record.y - wheel_pos.y) * 1/dt;
    odom.twist.twist.angular.z = (wheel_pos_record.th - wheel_pos.th) * 1/dt;

  	pub_odom.publish(odom);
}
void tfWheelOdomSender(const duckietown_msgs::WheelsCmdStamped::ConstPtr msg)
{
	//convert pulse to length(m)
	float dis_L = msg->vel_left / 4096 * WHEEL_RADIUS * 2.0 * 3.14 ;
	float dis_R = msg->vel_right / 4096 * WHEEL_RADIUS * 2.0 * 3.14 ;

	wheel_pos_record = wheel_pos ;
	// update wheel pose
	wheel_pos.th += (1/(WIDTH))*(dis_L-dis_R);
	wheel_pos.x += 0.5*(cos(wheel_pos.th)*dis_R+cos(wheel_pos.th)*dis_L);
	wheel_pos.y += 0.5*(sin(wheel_pos.th)*dis_R+sin(wheel_pos.th)*dis_L);
	wheel_pos.time = ros::Time::now();

	//cout << wheel_pos.x << " " << wheel_pos.y << " " << wheel_pos.th << "" << endl;

	// send transform
	tf::Transform transform;
	tf::Quaternion q;
	transform.setOrigin(tf::Vector3(wheel_pos.x, wheel_pos.y, 0));
  	q.setRPY(0 ,0, wheel_pos.th);
  	transform.setRotation(q);
  	try
  	{
  		br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom_frame"));
  	}
  	catch(tf::TransformException &ex)
  	{
		ROS_ERROR("transfrom exception : %s",ex.what());
  	}

	drawWheelOdom();

}
void cbWheelCmd(const duckietown_msgs::WheelsCmdStamped::ConstPtr& msg)
{
	tfWheelOdomSender(msg);
}
void cbLaserScan(const sensor_msgs::LaserScanStamped:;ConstPtr& msg)
{


}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheel_odometry_node");
	ros::NodeHandle nh("~");
	tf::TransformBroadcaster br_instance;
	br = &br_instance;

	//Initialize
	wheel_pos.x = 0;
	wheel_pos.y = 0;
	wheel_pos.th = 0;
	wheel_pos_record = wheel_pos ;

	nh.getParam("veh_name", bot_name);
	node_name = ros::this_node::getName();

	ros::Subscriber sub_wheel_cmd = nh.subscribe("wheel_cmd", 1000, cbWheelCmd);
	ros::Subscriber sub_laser_scan = nh.subscribe("/scan", 1000, cbLaserScan);
	pub_odom = nh.advertise<nav_msgs::Odometry>("wheel_odometry", 1000);

	ros::spin();

	return 0;
}
