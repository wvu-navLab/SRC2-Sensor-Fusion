#ifndef LOCALIZATION_WATCHDOG_H
#define LOCALIZATION_WATCHDOG_H

#include <math.h>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <termios.h>
#include <vector>

// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <std_msgs/Int64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <localization_watchdog/WatchdogStatus.h>
#include <driving_control/EnableDriving.h>

class LocalizationWatchdog
{
public:
  LocalizationWatchdog(ros::NodeHandle &);

  void WatchdogPublisher();

private:
  ros::NodeHandle &nh_;

  ros::Publisher pub_watchdog;

  ros::Subscriber sub_imu;
  ros::Subscriber sub_vo;
  ros::Subscriber sub_wo;
  ros::Subscriber sub_cmd_vel;
  ros::Subscriber sub_joint_states;
  ros::Subscriber sub_odometry;

  ros::ServiceClient clt_enable_driving;

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void voCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void woCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

  void ToggleDriving(bool enable);

  std::string robot_name_;

  sensor_msgs::Imu imu_msg_;
  double roll_curr_;
  double pitch_curr_;
  double yaw_curr_;

  nav_msgs::Odometry vo_msg_; 
  double vo_vx_;
  double vo_vy_;
  double vo_wz_;
  bool flag_vo_vx_;
  bool flag_vo_vy_;
  bool flag_vo_wz_;

  nav_msgs::Odometry wo_msg_;
  double wo_vx_;
  double wo_vy_;
  double wo_wz_;
  bool flag_wo_vx_;
  bool flag_wo_vy_;
  bool flag_wo_wz_;

  geometry_msgs::Twist cmd_vel_msg_;  
  double cmd_vx_;
  double cmd_vy_;
  double cmd_wz_;
  bool flag_cmd_vx_;
  bool flag_cmd_vy_;
  bool flag_cmd_wz_;

  sensor_msgs::JointState joint_state_msg_;
  double steering_fl_curr_;
  double steering_bl_curr_;
  double steering_fr_curr_;
  double steering_br_curr_;

  nav_msgs::Odometry odom_msg_;

  localization_watchdog::WatchdogStatus watchdog_msg_;
};

#endif //LOCALIZATION_WATCHDOG_H