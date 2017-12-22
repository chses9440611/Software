#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "duckietown_msgs/WheelsCmdStamped.h"

void cbWheelCmd(const duckietown_msgs::WheelsCmdStamped::ConstPtr& msg)
{
	printf("left = %f, right = %f", msg->vel_left, msg->vel_right);

}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheel_odometry_node");
	ros::NodeHandle nh("~");

	ros::Subscriber sub_wheel_cmd = nh.subscribe("wheel_cmd", 1000, cbWheelCmd);

	ros::spin();

	return 0 ;
}
