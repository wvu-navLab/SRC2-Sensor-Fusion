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

	subImu_ = nh_.subscribe("/scout_1/imu_filtered",10,&VIOAttitudeLocalization::imuCallback_,this);

	subWheelOdom_ = nh_.subscribe("/dead_reckoning/odometry",1,&VIOAttitudeLocalization::wheelOdomCallback_,this);

	pubOdom_ = nh_.advertise<nav_msgs::Odometry>("/scout_1/vio_attitude/odometry",1);

	double sigVel = .1;
	double sigPos = .01;
	Q_ << pow(sigPos,2), 0, 0, 0, 0, 0,
	      0, pow(sigPos,2), 0, 0, 0, 0,
	      0, 0, pow(sigPos,2), 0, 0, 0,
	      0, 0, 0, pow(sigVel,2), 0, 0,
	      0, 0, 0, 0, pow(sigVel,2), 0,
	      0, 0, 0, 0, 0, pow(sigVel,2);
	
	P_ = Q_;

	Hodom_ <<  0, 0, 0, 1, 0, 0,
	          0, 0, 0, 0, 1, 0,
		  0, 0, 0, 0, 0, 1;



	double sigWO = .05;
	double sigVIO = .05;
	Rwo_ << pow(sigWO,2), 0, 0,
	        0, pow(sigWO,2), 0,
                0, 0, pow(sigWO,2);
		

	Rvio_ << pow(sigVIO,2), 0, 0,
	         0, pow(sigVIO,2), 0,
		 0, 0, pow(sigVIO,2);

	F_ << 1, 0, 0, .2, 0, 0,
	      0, 1, 0, 0, .2, 0, 
	      0, 0, 1, 0, 0, .2,
	      0, 0, 0, 1, 0, 0,
	      0, 0, 0, 0, 1, 0,
	      0, 0, 0, 0, 0, 1;



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

void VIOAttitudeLocalization::wheelOdomCallback_(const nav_msgs::Odometry::ConstPtr& msg)
{


	R_body_imu_.setRPY(rollInc_/incCounter_, pitchInc_/incCounter_, yawInc_/incCounter_);
        rollInc_=0.0;
        pitchInc_=0.0;
        yawInc_=0.0;
        incCounter_=0.0;


	Rbn_=R_imu_nav_o_*R_body_imu_;

	double dt = msg->header.stamp.toSec() - lastTime_.toSec();
	
	// state predicition
        F_(0,3) = dt;
        F_(1,4) = dt;
        F_(2,5) = dt;

        x_ = F_*x_;
        P_ = F_*P_*F_.transpose()+ Q_;





	double roll, pitch, yaw;
        Rbn_.getRPY(roll,pitch,yaw);
        if((pitch*180/3.1414926) > 8){
                ROS_INFO("Skipping Wheel Odom Update due to High Pitch %f ",pitch*180/3.1414926);
        }else{


	tf::Vector3 vb_wo( msg->twist.twist.linear.x,
                               msg->twist.twist.linear.y,
                               msg->twist.twist.linear.z);




        // rotate kimeta body axis velocity into the nav frame
        tf::Vector3 vn_wo;
        vn_wo = Rbn_*vb_wo;



	// kimera measurement update
        zWO_(0,0)= vn_wo.x();
        zWO_(1,0)= vn_wo.y();
        zWO_(2,0)= vn_wo.z();
        Eigen::MatrixXd S(3,3);
        S = Rwo_ + Hodom_*P_*Hodom_.transpose();

        Eigen::MatrixXd K(6,3);
        K = (P_*Hodom_.transpose())*S.inverse();
        Eigen::MatrixXd I(6,6);
        I.setIdentity();
        P_ =(I-K*Hodom_)*P_;
        x_ = x_ + K*(zWO_ - Hodom_*x_);

	}

	nav_msgs::Odometry updatedOdom;

        updatedOdom.header.stamp = msg->header.stamp;
        updatedOdom.header.frame_id = "/scout_1_tf/odom";
        updatedOdom.child_frame_id = "/scout_1_tf/base_footprint";

        tf::Quaternion qup;
        qup.normalize();
        Rbn_.getRotation(qup);



        Rbn_.getRPY(roll,pitch,yaw);
        ROS_INFO("RPY in WO %f  %f  %f\n",roll*180.0/3.14,pitch*180.0/3.14,yaw*180.0/3.14);
        ROS_INFO_STREAM(" state x: " << x_.transpose() );
        updatedOdom.pose.pose.position.x = x_(0);
        updatedOdom.pose.pose.position.y = x_(1);
        updatedOdom.pose.pose.position.z = x_(2);

        updatedOdom.pose.pose.orientation.x = qup.x();
        updatedOdom.pose.pose.orientation.y = qup.y();
        updatedOdom.pose.pose.orientation.z = qup.z();
        updatedOdom.pose.pose.orientation.w = qup.w();

        updatedOdom.twist.twist.linear.x = x_(3);
        updatedOdom.twist.twist.linear.y = x_(4);
        updatedOdom.twist.twist.linear.z = x_(5);

        pubOdom_.publish(updatedOdom);
        pose_=updatedOdom.pose.pose;
        lastTime_=msg->header.stamp;



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

                tf::Vector3 vn_kimera;


	        // rotate kimeta body axis velocity into the nav frame
        	tf::Vector3 vn_imu;
                tf::Vector3 vb_kimera( msg->twist.twist.linear.x,
                               msg->twist.twist.linear.y,
                               msg->twist.twist.linear.z);
    
        	Rbn_=R_imu_nav_o_*R_body_imu_;
        	vn_imu = Rbn_*vb_kimera;

		x_ << pose_.position.x,
		      pose_.position.y,
		      pose_.position.z,
		      vn_imu.x(),
		      vn_imu.y(),
		      vn_imu.z();
	}

//	R_body_imu_.setRPY(rollInc_/incCounter_, pitchInc_/incCounter_, yawInc_/incCounter_);
//        rollInc_=0.0;
//        pitchInc_=0.0;
//        yawInc_=0.0;
//        incCounter_=0.0;
	
        // get kimera velocity represented in body frame

	tf::Vector3 vb_kimera( msg->twist.twist.linear.x,
			       msg->twist.twist.linear.y,
			       msg->twist.twist.linear.z);
	



	// rotate kimeta body axis velocity into the nav frame
	tf::Vector3 vn_kim;
//	Rbn_=R_imu_nav_o_*R_body_imu_;
	vn_kim = Rbn_*vb_kimera;


//	double dt = msg->header.stamp.toSec() - lastTime_.toSec();

     

	// state predicition
//	F_(0,3) = dt;
//	F_(1,4) = dt;
//	F_(2,5) = dt;

//	x_ = F_*x_;
//	P_ = F_*P_*F_.transpose()+ Q_;

	// kimera measurement update
	zVIO_(0,0)= vn_kim.x();
	zVIO_(1,0)= vn_kim.y();
        zVIO_(2,0)= vn_kim.z();
	Eigen::MatrixXd S(3,3);
	S = Rvio_ + Hodom_*P_*Hodom_.transpose();

	Eigen::MatrixXd K(6,3);
	K = (P_*Hodom_.transpose())*S.inverse();
        Eigen::MatrixXd I(6,6);
	I.setIdentity();
	P_ =(I-K*Hodom_)*P_;
	x_ = x_ + K*(zVIO_ - Hodom_*x_);


	// populate the odom message and publish
	/*
	nav_msgs::Odometry updatedOdom;

	updatedOdom.header.stamp = msg->header.stamp;
	updatedOdom.header.frame_id = "/scout_1_tf/odom";
	updatedOdom.child_frame_id = "/scout_1_tf/base_footprint";
	
	tf::Quaternion qup;
	qup.normalize();
	Rbn_.getRotation(qup);
	

        double roll, pitch, yaw;
        Rbn_.getRPY(roll,pitch,yaw);
        ROS_INFO("RPY in Kimera %f  %f  %f\n",roll*180.0/3.14,pitch*180.0/3.14,yaw*180.0/3.14);
	ROS_INFO_STREAM(" state x: " << x_.transpose() ); 
	updatedOdom.pose.pose.position.x = x_(0);
	updatedOdom.pose.pose.position.y = x_(1);
	updatedOdom.pose.pose.position.z = x_(2);

	updatedOdom.pose.pose.orientation.x = qup.x();
	updatedOdom.pose.pose.orientation.y = qup.y();
	updatedOdom.pose.pose.orientation.z = qup.z();
	updatedOdom.pose.pose.orientation.w = qup.w();

	updatedOdom.twist.twist.linear.x = x_(3);
	updatedOdom.twist.twist.linear.y = x_(4);
	updatedOdom.twist.twist.linear.z = x_(5);

	pubOdom_.publish(updatedOdom);
	pose_=updatedOdom.pose.pose;
	lastTime_=msg->header.stamp;
	*/
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

