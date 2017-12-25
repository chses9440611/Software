#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

#include "duckietown_msgs/WheelsCmdStamped.h"
struct Point
{
	float x;
	float y;
};
struct Position
{
	Point left;
	Point right;
	Point mid;
};
Position wheel_pos;
float agv_width = 50; //cm

ros::Publisher pub_odom ;
ros::Publisher pub_tf;

void drawWheelOdometry(const duckietown_msgs::WheelsCmdStamped::ConstPtr msg)
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

  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "agv"));
	
}
void cbWheelCmd(const duckietown_msgs::WheelsCmdStamped::ConstPtr& msg)
{
	drawWheelOdometry(msg);
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheel_odometry_node");

	wheel_pos.left.x = 0;
	wheel_pos.left.y = 0;
	wheel_pos.right.x = 0;
	wheel_pos.right.y = 0;
	wheel_pos.mid.x = 0;
	wheel_pos.mid.y = 0;

	ros::NodeHandle nh("~");
	
	ros::Subscriber sub_wheel_cmd = nh.subscribe("wheel_cmd", 1000, cbWheelCmd);
	pub_odom = nh.advertise<nav_msgs::Odometry>("wheel_odometry", 1000);
	ros::spin();

	return 0;
}
