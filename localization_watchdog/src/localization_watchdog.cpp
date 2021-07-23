#include "localization_watchdog/localization_watchdog.hpp"

LocalizationWatchdog::LocalizationWatchdog(ros::NodeHandle &nh) : nh_(nh)
{
  std::string node_name = "localization_watchdog";

  if (ros::param::get("robot_name", robot_name_) == false)
  {
    ROS_FATAL("No parameter 'robot_name' specified");
    ros::shutdown();
    exit(1);
  }

  sub_imu = nh_.subscribe("imu_filtered", 1, &LocalizationWatchdog::imuCallback, this);
  sub_vo = nh_.subscribe("vo", 1, &LocalizationWatchdog::voCallback, this);
  sub_wo = nh_.subscribe("dead_reckoning/odometry", 1, &LocalizationWatchdog::woCallback, this);
  sub_cmd_vel = nh_.subscribe("driving/cmd_vel", 1, &LocalizationWatchdog::cmdVelCallback, this);
  sub_joint_states = nh_.subscribe("joint_states", 1, &LocalizationWatchdog::jointStateCallback, this);
  sub_odometry = nh_.subscribe("localization/odometry/sensor_fusion", 1, &LocalizationWatchdog::odometryCallback, this);

  pub_watchdog = nh_.advertise<localization_watchdog::WatchdogStatus>("localization/watchdog", 1);
}

void LocalizationWatchdog::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  imu_msg_ = *msg;

  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                   msg->orientation.w);

  tf::Matrix3x3 m(q);

  m.getRPY(roll_curr_, pitch_curr_, yaw_curr_);
}

void LocalizationWatchdog::voCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  vo_msg_ = *msg;
  vo_vx_ = msg->twist.twist.linear.x;
  vo_vy_ = msg->twist.twist.linear.y;
  vo_wz_ = msg->twist.twist.angular.z;
  flag_vo_vx_ = (fabs(vo_vx_)>0.1);
  flag_vo_vy_ = (fabs(vo_vy_)>0.1);
  flag_vo_wz_ = (fabs(vo_wz_)>0.1);
}

void LocalizationWatchdog::woCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  wo_msg_ = *msg;
  wo_vx_ = msg->twist.twist.linear.x;
  wo_vy_ = msg->twist.twist.linear.y;
  wo_wz_ = msg->twist.twist.angular.z;
  flag_wo_vx_ = (fabs(wo_vx_)>0.1);
  flag_wo_vy_ = (fabs(wo_vy_)>0.1);
  flag_wo_wz_ = (fabs(wo_wz_)>0.1);
}

void LocalizationWatchdog::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd_vel_msg_ = *msg;
  cmd_vx_ = msg->linear.x;
  cmd_vy_ = msg->linear.y;
  cmd_wz_ = msg->angular.z;
  flag_cmd_vx_ = (fabs(cmd_vx_)>0.001);
  flag_cmd_vy_ = (fabs(cmd_vy_)>0.001);
  flag_cmd_wz_ = (fabs(cmd_wz_)>0.001);
}

void LocalizationWatchdog::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  joint_state_msg_ = *msg;

  int steering_fl_joint_idx;
  int steering_bl_joint_idx;
  int steering_fr_joint_idx;
  int steering_br_joint_idx;

  for (int i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == "fl_steering_arm_tibia_joint")
    {
      steering_fl_joint_idx = i;
    }
    if (msg->name[i] == "bl_steering_arm_tibia_joint")
    {
      steering_bl_joint_idx = i;
    }
    if (msg->name[i] == "fr_steering_arm_tibia_joint")
    {
      steering_fr_joint_idx = i;
    }
    if (msg->name[i] == "br_steering_arm_tibia_joint")
    {
      steering_br_joint_idx = i;
    }
  }

  steering_fl_curr_ = msg->position[steering_fl_joint_idx];
  steering_bl_curr_ = msg->position[steering_bl_joint_idx];
  steering_fr_curr_ = msg->position[steering_fr_joint_idx];
  steering_br_curr_ = msg->position[steering_br_joint_idx];
}

void LocalizationWatchdog::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  odom_msg_ = *msg;
}

void LocalizationWatchdog::WatchdogPublisher()
{
  watchdog_msg_.wasted.data = false;
  watchdog_msg_.immobile.data = false;

  // -----------------------------------------------------------------------
  // Check if the robots are flipped
  if(fabs(roll_curr_) > M_PI_2 || fabs(pitch_curr_) > M_PI_2)
  {
    watchdog_msg_.wasted.data = true;
  }

  if((flag_cmd_vx_ && !flag_vo_vx_) || (flag_cmd_vy_ && !flag_vo_vy_)  || (flag_cmd_wz_ && !flag_vo_wz_))
  {
    watchdog_msg_.immobile.data = true;
  }
  // -----------------------------------------------------------------------

  if (watchdog_msg_.wasted.data)
  {
    ROS_ERROR_STREAM("I'M WASTED!");
  }
  if (watchdog_msg_.immobile.data)
  {
    ROS_ERROR_STREAM("I'M IMMOBILE!");
  }
  pub_watchdog.publish(watchdog_msg_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization_watchdog");
  ros::NodeHandle nh("");

  ROS_INFO("Localization Watchdog node initializing.");
  ros::Rate rate(100);

  LocalizationWatchdog localization_watchdog(nh);

  while (ros::ok())
  {
    localization_watchdog.WatchdogPublisher();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
