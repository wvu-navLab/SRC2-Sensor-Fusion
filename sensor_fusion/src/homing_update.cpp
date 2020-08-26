#include "sensor_fusion/homing_update.h"

HomingUpdate::HomingUpdate(ros::NodeHandle & nh)
	: nh_(nh)
{
	firstCallback_=true;

	baseLocationClient_ = nh_.serviceClient<range_to_base::LocationOfBase>("location_of_base_service");
	homingUpdateServer_ = nh_.advertiseService("homing",&HomingUpdate::homingUpdate_,this);

	pubMeasurementUpdate_ = nh_.advertise<geometry_msgs::Pose>("position_update",10);
	


}

bool HomingUpdate::homingUpdate_(sensor_fusion::HomingUpdate::Request &req, sensor_fusion::HomingUpdate::Response &res){

	
	range_to_base::LocationOfBase srv;

	srv.request.angle = 0.4;

	baseLocationClient_.call(srv);
	
	if(req.initializeLandmark){
		 baseStationLocation_.x=srv.response.position.x;
                baseStationLocation_.y=srv.response.position.y;
                baseStationLocation_.z = 0.0;
                ROS_INFO(" Saving Base Station as Landmark x:%f y:%f", baseStationLocation_.x, baseStationLocation_.y);
		res.success=true;
		
	}
	else {
	   // our measurement is the relative error in x and y
                measurementUpdate_.position.x=baseStationLocation_.x - srv.response.position.x;
                measurementUpdate_.position.y=baseStationLocation_.y - srv.response.position.y;
                measurementUpdate_.position.z= baseStationLocation_.z - 0.0; // z isnt actually used...
                pubMeasurementUpdate_.publish(measurementUpdate_);
		res.success =true;

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

	while(ros::ok())
	{
		ros::spinOnce();
	}
}
