#include "sensor_fusion/imu_avg_velocity.hpp"

ImuAvgVelocity::ImuAvgVelocity(ros::NodeHandle & nh)
	: nh_(nh)
{
	sub_imu = nh_.subscribe("imu",10,&ImuAvgVelocity::imuCallback,this);
	sub_joint_states = nh_.subscribe("joint_states",10,&ImuAvgVelocity::jointStatesCallback,this);

    pub_twist = nh_.advertise<geometry_msgs::Twist>("localization/twist",1);
}

void ImuAvgVelocity::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

  int fl_wheel_joint_idx;
  int bl_wheel_joint_idx;
  int fr_wheel_joint_idx;
  int br_wheel_joint_idx;

  for (int i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == "fl_wheel_joint")
    {
      fl_wheel_joint_idx = i;
    }
    if (msg->name[i] == "bl_wheel_joint")
    {
      bl_wheel_joint_idx = i;
    }
    if (msg->name[i] == "fr_wheel_joint")
    {
      fr_wheel_joint_idx = i;
    }
    if (msg->name[i] == "br_wheel_joint")
    {
      br_wheel_joint_idx = i;
    }
  }


  if (fabs(msg->velocity[fl_wheel_joint_idx])<0.005 &&
  fabs(msg->velocity[bl_wheel_joint_idx])<0.005 &&
  fabs(msg->velocity[fr_wheel_joint_idx])<0.005 &&
  fabs(msg->velocity[br_wheel_joint_idx])<0.005)
  {
    avg_vx_ = 0.0;
    avg_vy_ = 0.0;
    avg_vz_ = 0.0;
  }
}


void ImuAvgVelocity::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    double avg_p_, avg_q_, avg_r_;
    double int_ax_, int_ay_, int_az_;

    if(imu_msgs_.size()>SET_SIZE-1)
    {
        imu_msgs_.erase(imu_msgs_.begin());
        imu_msgs_.push_back(*msg);

        for (auto &imu_msg : imu_msgs_)
        {
            avg_p_ += imu_msg.angular_velocity.x;
            avg_q_ += imu_msg.angular_velocity.y;
            avg_r_ += imu_msg.angular_velocity.z;
        }

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
    counter++;
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