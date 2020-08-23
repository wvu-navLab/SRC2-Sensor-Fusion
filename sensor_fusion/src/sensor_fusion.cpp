#include "sensor_fusion/sensor_fusion.h"



SensorFusion::SensorFusion(ros::NodeHandle & nh)
	: nh_(nh)
{

    std::string node_name = "sensor_fusion";
		std::string robot_name;



    if(ros::param::get(node_name+"/odometry_frame_id",odometry_frame_id)==false)
    {
      ROS_FATAL("No parameter 'odometry_frame_id' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/odometry_child_frame_id",odometry_child_frame_id)==false)
    {
      ROS_FATAL("No parameter 'odometry_child_frame_id' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/position_update_topic", position_update_topic)==false){
       ROS_FATAL("No parameter 'position_update_topic' specified");
       ros::shutdown();
       exit(1);
     }
     if(ros::param::get(node_name+"/robot_name", robot_name)==false){
        ROS_FATAL("No parameter 'robot_name' specified");
        ros::shutdown();
        exit(1);
      }

	clt_restart_kimera_ = nh.serviceClient<std_srvs::Trigger>("/kimera_vio_ros/kimera_vio_ros_node/restart_kimera_vio");

	averageIMU_ = true; // if true, IMU attitude will be averaged between wheel odom updates; if false latest IMU attitude is used
	firstVO_ = true;
	firstWO_ = true;
	firstIMU_= true;
	init_true_pose_=false;
	incCounter_=0;
	rollInc_=0;
	pitchInc_=0;
	yawInc_=0;
	// initialized_=NOT_INITIALIZED;

	subVO_=nh_.subscribe("vo",1, &SensorFusion::voCallback_, this);

	subImu_ = nh_.subscribe("imu_filtered",10,&SensorFusion::imuCallback_,this); //Robot namespace here

	subWheelOdom_ = nh_.subscribe("dead_reckoning/odometry",1,&SensorFusion::wheelOdomCallback_,this);

	subPositionUpdate_ = nh_.subscribe(position_update_topic, 1, &SensorFusion::positionUpdateCallback_,this);

	pubOdom_ = nh_.advertise<nav_msgs::Odometry>("localization/odometry/sensor_fusion",1); //Robot namespace here

	pubStatus_= nh_.advertise<std_msgs::Int64>("state_machine/localized_base_"+robot_name,100);


	double sigVel = .1;
	double sigPos = .01;
	Q_ << pow(sigPos,2), 0, 0, 0, 0, 0,
	      0, pow(sigPos,2), 0, 0, 0, 0,
	      0, 0, pow(sigPos,2), 0, 0, 0,
	      0, 0, 0, pow(sigVel,2), 0, 0,
	      0, 0, 0, 0, pow(sigVel,2), 0,
	      0, 0, 0, 0, 0, pow(sigVel,2);

	P_ = Q_;

	Hodom_ << 0, 0, 0, 1, 0, 0,
	          0, 0, 0, 0, 1, 0,
		  0, 0, 0, 0, 0, 1;

	Hposition_ << 1, 0, 0, 0, 0, 0,
		      0, 1, 0, 0, 0, 0,
		      0, 0, 1, 0, 0, 0;

	double sigPosition = .2;
	Rposition_ << pow(sigPosition,2), 0, 0,
		      0, pow(sigPosition,2), 0,
		      0, 0, pow(sigPosition,2);


	double sigWO = .05;
	double sigVO = .15;
	Rwo_ << pow(sigWO,2), 0, 0,
	        0, pow(sigWO,2), 0,
                0, 0, pow(sigWO,2);


	Rvo_ << pow(sigVO,2), 0, 0,
	         0, pow(sigVO,2), 0,
		 0, 0, pow(sigVO,2);

	F_ << 1, 0, 0, .2, 0, 0,
	      0, 1, 0, 0, .2, 0,
	      0, 0, 1, 0, 0, .2,
	      0, 0, 0, 1, 0, 0,
	      0, 0, 0, 0, 1, 0,
	      0, 0, 0, 0, 0, 1;



}

void SensorFusion::imuCallback_(const sensor_msgs::Imu::ConstPtr& msg)
{
	// std::cout << " IMU Callback " << std::endl;
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

	if(averageIMU_){

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
	}
	else{
		// not averaging, just using current roll, pitch, yaw
		rollInc_ = roll;
		pitchInc_ = pitch;
		yawInc_ = yaw;
	}

	firstIMU_=false;
//	ROS_INFO("RPY %f  %f  %f\n",roll*180.0/3.14,pitch*180.0/3.14,yaw*180.0/3.14);
}

void SensorFusion::wheelOdomCallback_(const nav_msgs::Odometry::ConstPtr& msg)
{


	tf::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w
                    );

        tf::Matrix3x3 R_wo_b_n(q);




        if(firstWO_){
                if(!firstIMU_) firstWO_=false;
                //since kimera is init with true pose, pick up true global attitude on first call
                // later we can get this directly from get true pose


                R_imu_nav_o_ = R_wo_b_n*R_body_imu_.transpose();

                pose_ = msg->pose.pose;


                // rotate kimeta body axis velocity into the nav frame
                tf::Vector3 vn_imu;
                tf::Vector3 vb_wo( msg->twist.twist.linear.x,
                               msg->twist.twist.linear.y,
                               msg->twist.twist.linear.z);

                Rbn_=R_imu_nav_o_*R_body_imu_;
                vn_imu = Rbn_*vb_wo;

                x_ << pose_.position.x,
                      pose_.position.y,
                      pose_.position.z,
                      vn_imu.x(),
                      vn_imu.y(),
                      vn_imu.z();

											// if (!isnan(x_(0))) {
												init_true_pose_=true; // TODO: True pose should be called directly from the service! This is temporary.
											// }

        }


	// std::cout <<"Wheel Odom Callback " << std::endl;
	if(averageIMU_){
		R_body_imu_.setRPY(rollInc_/incCounter_, pitchInc_/incCounter_, yawInc_/incCounter_);
		rollInc_=0.0;
        	pitchInc_=0.0;
        	yawInc_=0.0;
        	incCounter_=0.0;
	}
	else{
		// not averaging, just use current roll, pitch & yaw
		R_body_imu_.setRPY(rollInc_, pitchInc_, yawInc_);
	}


	Rbn_=R_imu_nav_o_*R_body_imu_;

	double dt = msg->header.stamp.toSec() - lastTime_wo_.toSec();

	// state predicition
        F_(0,3) = dt;
        F_(1,4) = dt;
        F_(2,5) = dt;

        x_ = F_*x_;
        P_ = F_*P_*F_.transpose()+ Q_;



	double roll, pitch, yaw;
        Rbn_.getRPY(roll,pitch,yaw);
        if((pitch*180/3.1414926) > 90){
                ROS_INFO("Skipping Wheel Odom Update due to High Pitch %f ",pitch*180/3.1414926);
        }
	else{


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



        lastTime_wo_=msg->header.stamp;
	publishOdom_();


	if((fabs(lastTime_wo_.toSec()-lastTime_vo_.toSec())>60)&& !firstVO_){
		ROS_INFO_STREAM(" VO NODE FAIL!? " );
		ROS_INFO_STREAM(" lastTime_wo " <<lastTime_wo_.toSec() );
		ROS_INFO_STREAM(" lastTime_vio " <<lastTime_vo_.toSec() );
		ROS_INFO_STREAM(" dt" <<fabs(lastTime_wo_.toSec()-lastTime_vo_.toSec()) );
		std_srvs::Trigger trig;
		clt_restart_kimera_.call(trig);
		lastTime_vo_=msg->header.stamp;


	}

	//TODO: if pose NaN, then publish lost state message



}

void SensorFusion::positionUpdateCallback_(const geometry_msgs::Pose::ConstPtr& msg){

	 
        zPosition_(0,0)= msg->position.x;
        zPosition_(1,0)= msg->position.y;
        zPosition_(2,0)= msg->position.z;
        Eigen::MatrixXd S(3,3);
        S = Rposition_ + Hposition_*P_*Hposition_.transpose();

        Eigen::MatrixXd K(6,3);
        K = (P_*Hposition_.transpose())*S.inverse();
        Eigen::MatrixXd I(6,6);
        I.setIdentity();
        P_ =(I-K*Hposition_)*P_;
        x_ = x_ + K*(zPosition_ - Hposition_*x_);


}



void SensorFusion::voCallback_(const nav_msgs::Odometry::ConstPtr& msg)
{

	if(!firstIMU_){
		firstVO_=false;

	}

	std::cout << " WVU Callback " << std::endl;
	//TODO: if pose NaN, then stop the rover, restart kimera with the latest pose (the one before NaN)
	tf::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w
                    );



        // get kimera velocity represented in body frame

	tf::Vector3 vb_kimera( msg->twist.twist.linear.x,
			       msg->twist.twist.linear.y,
			       msg->twist.twist.linear.z);




	// rotate kimera body axis velocity into the nav frame
	tf::Vector3 vn_kim;
	vn_kim = Rbn_*vb_kimera;


	// kimera measurement update
	zVO_(0,0)= vn_kim.x();
	zVO_(1,0)= vn_kim.y();
        zVO_(2,0)= vn_kim.z();
	Eigen::MatrixXd S(3,3);
	S = Rvo_ + Hodom_*P_*Hodom_.transpose();

	Eigen::MatrixXd K(6,3);
	K = (P_*Hodom_.transpose())*S.inverse();
        Eigen::MatrixXd I(6,6);
	I.setIdentity();
	Eigen::Vector3d Innovation;

	Innovation = zVO_ - Hodom_*x_;

	lastTime_vo_ = msg->header.stamp;

	if (Innovation.norm()>2.0) {
		ROS_INFO_STREAM(" VO UPDATE SKIP DUE TO VELOCITY ERROR! " );
		return;
	}
	P_ =(I-K*Hodom_)*P_;
	x_ = x_ + K*(zVO_ - Hodom_*x_);


}

void SensorFusion::initializationStatus_()
{
 std_msgs::Int64 status;
	if (init_true_pose_) {
		status.data=INITIALIZED;
	}
	else {
		status.data=NOT_INITIALIZED;
	}

        pubStatus_.publish(status);

}

void SensorFusion::publishOdom_()
{
	nav_msgs::Odometry updatedOdom;

        updatedOdom.header.stamp = lastTime_wo_;
        updatedOdom.header.frame_id = odometry_frame_id;
        updatedOdom.child_frame_id = odometry_child_frame_id;

        tf::Quaternion qup;
        qup.normalize();
        Rbn_.getRotation(qup);


	double roll, pitch, yaw;
        Rbn_.getRPY(roll,pitch,yaw);
        // ROS_INFO("RPY in WO %f  %f  %f\n",roll*180.0/3.14,pitch*180.0/3.14,yaw*180.0/3.14);
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


	geometry_msgs::TransformStamped odom_trans;
    	odom_trans.header.stamp = updatedOdom.header.stamp ;
    	odom_trans.header.frame_id = updatedOdom.header.frame_id;
    	odom_trans.child_frame_id = updatedOdom.child_frame_id ;
    	odom_trans.transform.translation.x = updatedOdom.pose.pose.position.x;
    	odom_trans.transform.translation.y = updatedOdom.pose.pose.position.y;
    	odom_trans.transform.translation.z = updatedOdom.pose.pose.position.z;
    	odom_trans.transform.rotation.x =updatedOdom.pose.pose.orientation.x ;
    	odom_trans.transform.rotation.y =updatedOdom.pose.pose.orientation.y ;
    	odom_trans.transform.rotation.z =updatedOdom.pose.pose.orientation.z ;
    	odom_trans.transform.rotation.w =updatedOdom.pose.pose.orientation.w ;
    	odom_broadcaster_.sendTransform(odom_trans);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensor_fusion");
	ros::NodeHandle nh("");
	ros::Rate rate(100);
	ROS_INFO(" Sensor Fusion started ");

	SensorFusion localizer(nh);

	// ros::spin();
	while(ros::ok())
	{

			localizer.initializationStatus_();
			ros::spinOnce();
			rate.sleep();
	}

	return 0;
}
