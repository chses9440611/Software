#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "duckietown_msgs/WheelsCmdStamped.h"
struct Point
{
	float x;
	float y;
};
Point wheel_pos;
tf::TransformListener listener;
float agv_width = 50; //cm
std::String bot_name;
std::String node_name;
ros::Publisher pub_odom;
void tfWheelOdometryListner()
{
	tf::Transform transform;
	ros::Time now = ros::Time::now();
	try{
		listener.waitForTransform("/world", bot_name,
	                              now, ros::Duration(3.0));
	    listener.lookupTransform("/world", bot_name,
	                             now, transform);
	}
	

}
void tfWheelOdometrySender(const duckietown_msgs::WheelsCmdStamped::ConstPtr msg)
{
	//convert pulse to length(m)
	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	float left = msg->vel_left / 4096 * 8.5 * 2 * 3.14 / 100;
	float right = msg->vel_right / 4096 * 8.5 * 2 * 3.14 / 100;
	float r = (left < right) ? (-0.5 * left / right) : (0.5 * right / left);
	float theta = left / r ; //(rad)
	if (r == 1)
	{
		theta = 0;
	}
	
	transform.setOrigin(tf::Vector3(wheel_pos.mid.x, wheel_pos.mid.y, 0));
  	q.setRPY(0 ,0, theta);
  	transform.setRotation(q);

  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", bot_name));

	tfWheelOdometryListner();
}
void cbWheelCmd(const duckietown_msgs::WheelsCmdStamped::ConstPtr& msg)
{
	tfWheelOdometrySender(msg);
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheel_odometry_node");
	bot_name = ros::param:get_param("veh_name", bot_name);
	node_name = ros::get_name();
	wheel_pos.x = 0;
	wheel_pos.y = 0;

	ros::NodeHandle nh("~");
	
	ros::Subscriber sub_wheel_cmd = nh.subscribe("wheel_cmd", 1000, cbWheelCmd);
	pub_odom = nh.advertise<nav_msgs::Odometry>("wheel_odometry", 1000);

	while ()
	ros::spin();

	return 0;
}
