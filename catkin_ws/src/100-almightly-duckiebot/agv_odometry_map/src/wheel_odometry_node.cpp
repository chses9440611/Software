#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "duckietown_msgs/WheelsCmdStamped.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

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

Point wheel_state, wheel_state_record;
std::string bot_name;
std::string node_name;
ros::Publisher pub_odom;
tf::TransformBroadcaster* br;
void drawWheelOdom(const duckietown_msgs::WheelsCmdStamped::ConstPtr msg)
{
	nav_msgs::Odometry odom;
	odom.header.stamp = msg->header.stamp;
	odom.header.frame_id = "map";
	odom.child_frame_id = "odom_frame";

	odom.pose.pose.position.x = wheel_state.x;
  	odom.pose.pose.position.y = wheel_state.y;
  	odom.pose.pose.position.z = 0.0;
  	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(wheel_state.th);

  	//cal speed 
  	float dt = wheel_state.time.toSec() - wheel_state_record.time.toSec();
  	if (dt != 0)	
  	{
  		odom.twist.twist.linear.x = (wheel_state.x - wheel_state_record.x) * 1/dt;
    	odom.twist.twist.linear.y = (wheel_state.y - wheel_state_record.y) * 1/dt;
    	odom.twist.twist.angular.z = (wheel_state.th - wheel_state_record.th) * 1/dt;
  		pub_odom.publish(odom);
  	}

}
void tfWheelOdomSender(const duckietown_msgs::WheelsCmdStamped::ConstPtr msg)
{
	//convert pulse to length(m)
	float dis_L = msg->vel_left / 4096 * WHEEL_RADIUS * 2.0 * 3.14 ;
	float dis_R = msg->vel_right / 4096 * WHEEL_RADIUS * 2.0 * 3.14 ;

	wheel_state_record = wheel_state ;
	// update wheel pose
	wheel_state.th += (1/(WIDTH))*(dis_L-dis_R);
	wheel_state.x += 0.5*(cos(wheel_state.th)*dis_R+cos(wheel_state.th)*dis_L);
	wheel_state.y += 0.5*(sin(wheel_state.th)*dis_R+sin(wheel_state.th)*dis_L);
	wheel_state.time = msg->header.stamp;

	//cout << wheel_state.x << " " << wheel_state.y << " " << wheel_state.th << "" << endl;

	// send transform
	tf::Transform transform;
	tf::Quaternion q;
	transform.setOrigin(tf::Vector3(wheel_state.x, wheel_state.y, 0));
  	q.setRPY(0 ,0, wheel_state.th);
  	transform.setRotation(q);
  	try
  	{
  		br->sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "base_link"));

  	}
  	catch(tf::TransformException &ex)
  	{
		ROS_ERROR("transfrom exception : %s",ex.what());
  	}
	drawWheelOdom(msg);
	
}
void cbWheelCmd(const duckietown_msgs::WheelsCmdStamped::ConstPtr& msg)
{
	tfWheelOdomSender(msg);
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheel_odometry_node");
	ros::NodeHandle nh("~");
	tf::TransformBroadcaster br_instance;
	br = &br_instance;

	//Initialize wheel position
	wheel_state.x = 0.0;
	wheel_state.y = 0.0;
	wheel_state.th = 0.0;
	wheel_state_record = wheel_state ;

	nh.getParam("veh_name", bot_name);
	node_name = ros::this_node::getName();

	ros::Subscriber sub_wheel_cmd = nh.subscribe("wheel_cmd", 1000, cbWheelCmd);
	pub_odom = nh.advertise<nav_msgs::Odometry>("wheel_odometry", 1000);


	ros::spin();

	return 0;
}
