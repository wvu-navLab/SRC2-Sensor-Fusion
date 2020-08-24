#ifndef HOMING_UPDATE_H
#define HOMING_UPDATE_H

#include <math.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <termios.h>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>


// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>


class HomingUpdate
{
public:
	HomingUpdate(ros::NodeHandle &);

private:

	ros::NodeHandle & nh_;
	bool firstCallback_;

	ros::Publisher pubMeasurementUpdate_;
	ros::Subscriber subBaseLocation_;

	geometry_msgs::Point baseStationLocation_;
	geometry_msgs::Pose measurementUpdate_;

	void baseLocationCallback_(const geometry_msgs::Point::ConstPtr& msg);
};

#endif

