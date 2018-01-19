#!/usr/bin/env bash
IP=$(ifconfig  | grep 'inet addr:'| grep -v '127.0.0.1' | grep -v '10.42.0.1' | cut -d: -f2 | awk '{ print $1}')
echo "Setting ROS_IP..."

if [ $# -gt 0 ]; then
	# provided a hostname, use it as ROS_MASTER_URI
	export ROS_IP=$1
else
	echo "No ROS_IP provided. Using $IP."
	export ROS_IP=$IP
fi
echo "ROS_IP set to $ROS_IP"