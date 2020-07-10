#include "vio_attitude_localization/vio_attitude_localization.h"


VIOAttitudeLocalization::VIOAttitudeLocalization(ros::NodeHandle & nh)
	: nh_(nh)
{

	firstKimera_ = true;
	firstIMU_= true;
	incCounter_=0;
	rollInc_=0;
	pitchInc_=0;
	yawInc_=0;

	subKimera_=nh_.subscribe("/kimera_vio_ros/odometry",1, &VIOAttitudeLocalization::kimeraCallback_, this);

	subImu_ = nh_.subscribe("/scout_1/imu",10,&VIOAttitudeLocalization::imuCallback_,this);

	pubOdom_ = nh_.advertise<nav_msgs::Odometry>("/scout_1/vio_attitude/odometry",1);
}

void VIOAttitudeLocalization::imuCallback_(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf::Quaternion q(
                    msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w
                    );


	tf::Matrix3x3 m(q);

	
	
	double roll, pitch, yaw;
	m.getRPY(roll,pitch,yaw);
	if(firstIMU_) R_body_imu_ =m;
	rollInc_ = rollInc_ + roll;
	pitchInc_ = pitchInc_ + pitch;
	incCounter_ = incCounter_ + 1.0;
	if(incCounter_ > 1.0){
		// if there has been a roll over
		double yawAvg=(yawInc_/(incCounter_-1.0));
		if (fabs( yaw - yawAvg)>3.14){
			std::cout << "Roll Over Detected " << yaw << " " << yawAvg << std::endl;
			if( yaw < yawAvg ) yaw = yaw +6.2831185;
			else yaw = yaw - 6.2831185;
		}
	}


	yawInc_ = yawInc_ + yaw;
	firstIMU_=false;
//	ROS_INFO("RPY %f  %f  %f\n",roll*180.0/3.14,pitch*180.0/3.14,yaw*180.0/3.14);
}

void VIOAttitudeLocalization::kimeraCallback_(const nav_msgs::Odometry::ConstPtr& msg)
{


	
		
        tf::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w
                    );

        tf::Matrix3x3 R_kimera_b_n(q);



	if(firstKimera_){
		if(!firstIMU_) firstKimera_=false;
		//since kimera is init with true pose, pick up true global attitude on first call
		// later we can get this directly from get true pose

		lastTime_ = msg->header.stamp;
		R_imu_nav_o_ = R_kimera_b_n*R_body_imu_.transpose();

		pose_ = msg->pose.pose;
	}
	ROS_INFO("Avg IMU %f %f %f %f",(rollInc_/incCounter_)*180.0/3.14, (pitchInc_/incCounter_)*180.0/3.14, (yawInc_/incCounter_)*180.0/3.14, incCounter_);
	R_body_imu_.setRPY(rollInc_/incCounter_, pitchInc_/incCounter_, yawInc_/incCounter_);
        rollInc_=0.0;
        pitchInc_=0.0;
        yawInc_=0.0;
        incCounter_=0.0;
	

	tf::Vector3 vn_kimera( msg->twist.twist.linear.x,
			       msg->twist.twist.linear.y,
			       msg->twist.twist.linear.z);

	// get kimera velocity represented in body frame
	
	tf::Vector3 vb_kimera;

	vb_kimera = vn_kimera;// R_kimera_b_n.transpose()*vn_kimera;

	// rotate kimeta body axis velocity into the nav frame
	tf::Vector3 vn_imu;
	tf::Matrix3x3 R;
	R=R_imu_nav_o_*R_body_imu_;
	//R=R_body_imu_;
	vn_imu = R*vb_kimera;
	ROS_INFO_STREAM("Vn kimera " << vn_kimera.x() << " " << vn_kimera.y() << " " << vn_kimera.z() );
	ROS_INFO_STREAM("Vb kimera " << vb_kimera.x()<< " " << vb_kimera.y() << " " << vb_kimera.z() );
	ROS_INFO_STREAM("Vn IMU " << vn_imu.x() << " " << vn_imu.y() << " " << vn_imu.z() );
	//integrate position over time
	tf::Vector3 currentPosition( pose_.position.x,
			         pose_.position.y,
				 pose_.position.z);

	double dt = msg->header.stamp.toSec() - lastTime_.toSec();
        ROS_INFO(" In KimeraCB dt %f ",dt);
	tf::Vector3 integratedPosition;
	tf::Vector3 v_clamp(1.0,1.0,.1);
	integratedPosition = currentPosition + v_clamp*vn_imu*dt;

	// populate the odom message and publish
	
	nav_msgs::Odometry updatedOdom;

	updatedOdom.header.stamp = msg->header.stamp;
	updatedOdom.header.frame_id = "/scout_1_tf/odom";
	updatedOdom.child_frame_id = "/scout_1_tf/base_footprint";
	
	tf::Quaternion qup;
	qup.normalize();
	R.getRotation(qup);
	//tf::Vector3 rotAxis = qup.getAxis();

        double roll, pitch, yaw;
        R.getRPY(roll,pitch,yaw);
        ROS_INFO("RPY in Kimera %f  %f  %f\n",roll*180.0/3.14,pitch*180.0/3.14,yaw*180.0/3.14);

	updatedOdom.pose.pose.position.x = integratedPosition.x();
	updatedOdom.pose.pose.position.y = integratedPosition.y();
	updatedOdom.pose.pose.position.z = integratedPosition.z();

	updatedOdom.pose.pose.orientation.x = qup.x();
	updatedOdom.pose.pose.orientation.y = qup.y();
	updatedOdom.pose.pose.orientation.z = qup.z();
	updatedOdom.pose.pose.orientation.w = qup.w();

	updatedOdom.twist.twist.linear.x = vn_imu.x();
	updatedOdom.twist.twist.linear.y = vn_imu.y();
	updatedOdom.twist.twist.linear.z = vn_imu.z();

	pubOdom_.publish(updatedOdom);
	pose_=updatedOdom.pose.pose;
	lastTime_=msg->header.stamp;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vio_attitude_localization");
	ros::NodeHandle nh("");

	ROS_INFO(" VIO Localization started ");

	VIOAttitudeLocalization localizer(nh);

	ros::spin();

	return 0;
}

