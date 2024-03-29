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
#include <sensor_fusion/HomingUpdate.h>
#include <sensor_fusion/HomingUpdateProcessingPlant.h>
#include <sensor_fusion/SetBaseLocation.h>
#include <range_to_base/LocationOfBase.h>
#include <range_to_base/LocationOfProcessingPlant.h>

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
	ros::Publisher pubBaseLocation_;
	ros::Subscriber subBaseLocation_;
	ros::ServiceServer homingUpdateServer_;
	ros::ServiceServer homingUpdateProcessingPlantServer_;
	ros::ServiceServer setBaseLocationServer_;

	ros::ServiceClient baseLocationClient_;
	ros::ServiceClient processingPlantLocationClient_;

	geometry_msgs::Point baseStationLocation_;
	geometry_msgs::Pose measurementUpdate_;
	geometry_msgs::Point processingPlantLocation_;
	geometry_msgs::Pose measurementUpdateProcessingPlant_;


	bool homingUpdate_(sensor_fusion::HomingUpdate::Request &req, sensor_fusion::HomingUpdate::Response &res);
	bool homingUpdateProcessingPlant_(sensor_fusion::HomingUpdateProcessingPlant::Request &req, sensor_fusion::HomingUpdateProcessingPlant::Response &res);
	bool setBaseLocation_(sensor_fusion::SetBaseLocation::Request &req, sensor_fusion::SetBaseLocation::Response &res);

	void baseLocationCallback_(const geometry_msgs::Point::ConstPtr& msg);
	void processingPlantLocationCallback_(const geometry_msgs::Point::ConstPtr& msg);

};

#endif
