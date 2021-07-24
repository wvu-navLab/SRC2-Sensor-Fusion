#include "sensor_fusion/imu_avg_velocity.hpp"

ImuAvgVelocity::ImuAvgVelocity(ros::NodeHandle & nh)
	: nh_(nh)
{
	sub_imu = nh_.subscribe("imu",10,&ImuAvgVelocity::imuCallback,this);

    pub_twist = nh_.advertise<geometry_msgs::Twist>("localization/twist",1);
}

void ImuAvgVelocity::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    double avg_vx_, avg_vy_, avg_vz_;
    double avg_p_, avg_q_, avg_r_;

    if(imu_msgs_.size()>49)
    {
        imu_msgs_.erase(imu_msgs_.begin());
        imu_msgs_.push_back(*msg);
        int i = 0;
        for (auto &imu_msg : imu_msgs_)
        {
            if (i==0 || i==SET_SIZE-1)
            {
                avg_vx_ += 0.5*imu_msg.linear_acceleration.x;
                avg_vy_ += 0.5*imu_msg.linear_acceleration.y;
                avg_vz_ += 0.5*(imu_msg.linear_acceleration.z - MOON_GRAVITY);
            }
            else
            {
                avg_vx_ += imu_msg.linear_acceleration.x;
                avg_vy_ += imu_msg.linear_acceleration.y;
                avg_vz_ += imu_msg.linear_acceleration.z;
            }
            avg_p_ += imu_msg.angular_velocity.x;
            avg_q_ += imu_msg.angular_velocity.y;
            avg_r_ += imu_msg.angular_velocity.z;
            i++;
        }

        avg_vx_ = avg_vx_ * imu_dt_;
        avg_vy_ = avg_vy_ * imu_dt_;
        avg_vz_ = avg_vz_ * imu_dt_;
        avg_p_ = avg_p_ / (double) SET_SIZE;
        avg_q_ = avg_q_ / (double) SET_SIZE;
        avg_r_ = avg_r_ / (double) SET_SIZE;
                
        geometry_msgs::Twist twist;
        twist.linear.x = avg_vx_;
        twist.linear.y = avg_vy_;
        twist.linear.z = avg_vz_;
        twist.angular.x = avg_p_;
        twist.angular.y = avg_q_;
        twist.angular.z = avg_r_;
        
        pub_twist.publish(twist);
    }
    else
    {
        imu_msgs_.push_back(*msg);
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_avg_velocity");
	ros::NodeHandle nh("");

	ROS_INFO("IMU Velocity estimator started.");

	ImuAvgVelocity imu_avg_velocity(nh);

	ros::spin();

	return 0;
}