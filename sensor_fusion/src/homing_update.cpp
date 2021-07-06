#include "sensor_fusion/homing_update.h"

HomingUpdate::HomingUpdate(ros::NodeHandle & nh)
	: nh_(nh)
{
	firstCallback_=true;

	baseLocationClient_ = nh_.serviceClient<range_to_base::LocationOfBase>("location_of_base_service");
	homingUpdateServer_ = nh_.advertiseService("homing",&HomingUpdate::homingUpdate_,this);
	setBaseLocationServer_ = nh_.advertiseService("set_base_location",&HomingUpdate::setBaseLocation_,this);

	pubMeasurementUpdate_ = nh_.advertise<geometry_msgs::Pose>("position_update",10);
	pubBaseLocation_ = nh_.advertise<geometry_msgs::Point>("/base_station",10);

	subBaseLocation_ = nh_.subscribe(
			"/base_station", 1, &HomingUpdate::baseLocationCallback_, this);

}
void HomingUpdate::baseLocationCallback_(const geometry_msgs::Point::ConstPtr& msg){

	baseStationLocation_.x = msg->x;
	baseStationLocation_.y = msg->y;
	baseStationLocation_.z = 0.0;

	ROS_INFO(" Saving Base Station as Landmark x:%f y:%f", baseStationLocation_.x, baseStationLocation_.y);
}
bool HomingUpdate::setBaseLocation_(sensor_fusion::SetBaseLocation::Request &req, sensor_fusion::SetBaseLocation::Response &res){

	baseStationLocation_ = req.base_location;
	res.success = true;
	return true;
}

bool HomingUpdate::homingUpdate_(sensor_fusion::HomingUpdate::Request &req, sensor_fusion::HomingUpdate::Response &res){


	range_to_base::LocationOfBase srv;

	srv.request.angle = req.angle;
	while (!baseLocationClient_.waitForExistence())
	{
			ROS_WARN("HOMING: Waiting for Location of Base Service");
	}
	baseLocationClient_.call(srv);


	if(req.initializeLandmark)
	{
		if(fabs(srv.response.position.x) < 100.0 && fabs(srv.response.position.y) < 100)
		{
			baseStationLocation_.x=srv.response.position.x;
			baseStationLocation_.y=srv.response.position.y;
			baseStationLocation_.z = 0.0;
			ROS_INFO(" Saving Base Station as Landmark x:%f y:%f", baseStationLocation_.x, baseStationLocation_.y);
			pubBaseLocation_.publish(baseStationLocation_);
			res.success=true;
		}
		else
		{
			res.success=false;
		}

	}
	else {
		if(fabs(srv.response.position.x) < 100.0 && fabs(srv.response.position.y) < 100)
		{
			// our measurement is the relative error in x and y
			measurementUpdate_.position.x=baseStationLocation_.x - srv.response.position.x;
			measurementUpdate_.position.y=baseStationLocation_.y - srv.response.position.y;
			measurementUpdate_.position.z=baseStationLocation_.z - 0.0; // z isnt actually used...
			pubMeasurementUpdate_.publish(measurementUpdate_);
			res.success =true;
		}
		else
		{
			res.success = false;
		}

	}

	//also returning the estimate of the base station location
	res.base_location = baseStationLocation_;

	return true;
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "homing_update");
	ros::NodeHandle nh("");
	ROS_INFO(" Homing Update Started ");

	HomingUpdate homing(nh);

	ros::Rate rate(10);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
}
