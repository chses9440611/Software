#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "duckietown_msgs/WheelsCmdStamped.h"
struct distance
{
	float dis_left;
	float dis_right;
};
distance wheel;
float agv_width = 50; //cm
ros::Publisher pub_odom ;
void drawWheelOdometry(const duckietown_msgs::WheelsCmdStamped::ConstPtr msg)
{
	float left = msg->vel_left;
	float right = msg->vel_right;
	//convert pulse to length(cm)
	wheel.dis_left = left / 4096 * 8.5 * 2 * 3.14 ;	
	wheel.dis_right = right / 4096 * 8.5 * 2 * 3.14 ;


	nav_msgs::Odometry odom;
	odom.header.stamp = msg->header.stamp;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = 1;
    odom.pose.pose.position.y = 2;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = 1;

    odom.twist.twist.linear.x = 0.1;
    odom.twist.twist.linear.y = 0.1;
    odom.twist.twist.angular.z = 0;
  

	pub_odom.publish(odom);
}
void cbWheelCmd(const duckietown_msgs::WheelsCmdStamped::ConstPtr& msg)
{
	drawWheelOdometry(msg);
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheel_odometry_node");
	ros::NodeHandle nh("~");
	
	ros::Subscriber sub_wheel_cmd = nh.subscribe("wheel_cmd", 1000, cbWheelCmd);
	
	pub_odom = nh.advertise<nav_msgs::Odometry>("wheel_odometry", 1000);
	ros::spin();

	return 0;
}
