#include "sensor_fusion/homing_update.h"

HomingUpdate::HomingUpdate(ros::NodeHandle & nh)
	: nh_(nh)
{
	firstCallback_=true;

	pubMeasurementUpdate_ = nh_.advertise<geometry_msgs::Pose>("position_update",10);
	subBaseLocation_ = nh_.subscribe("base_location",10, &HomingUpdate::baseLocationCallback_, this);


}

void HomingUpdate::baseLocationCallback_(const geometry_msgs::Point::ConstPtr& msg){

	if(firstCallback_){
		
		baseStationLocation_.x=msg->x;
		baseStationLocation_.y=msg->y;
		baseStationLocation_.z = 0.0;
		ROS_INFO(" Saving Base Station as Landmark x:%f y:%f", baseStationLocation_.x, baseStationLocation_.y);
		firstCallback_= false;
	}else{

		// our measurement is the relative error in x and y
		measurementUpdate_.position.x=baseStationLocation_.x - msg->x;
		measurementUpdate_.position.y=baseStationLocation_.y - msg->y;
		measurementUpdate_.position.z= baseStationLocation_.z - msg->z; // z isnt actually used...
		pubMeasurementUpdate_.publish(measurementUpdate_);

	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "homing_update");
	ros::NodeHandle nh("");
	ROS_INFO(" Homing Update Started ");

	HomingUpdate homing(nh);

	while(ros::ok())
	{
		ros::spinOnce();
	}
}
