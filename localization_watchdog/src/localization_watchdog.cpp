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

  if (ros::param::get(node_name + "/immobility_threshold", immobility_threshold) == false)
  {
    ROS_FATAL("No parameter 'immobility_threshold' specified");
    ros::shutdown();
    exit(1);
  }

  sub_imu = nh_.subscribe("imu_filtered", 1, &LocalizationWatchdog::imuCallback, this);
  sub_vo = nh_.subscribe("localization/odometry/vo", 1, &LocalizationWatchdog::voCallback, this);
  sub_wo = nh_.subscribe("localization/odometry/dead_reckoning", 1, &LocalizationWatchdog::woCallback, this);
  sub_cmd_vel = nh_.subscribe("driving/cmd_vel", 1, &LocalizationWatchdog::cmdVelCallback, this);
  sub_cmd_steering = nh_.subscribe("control/steering/joint_angles", 1, &LocalizationWatchdog::cmdSteeringCallback, this);
  sub_joint_states = nh_.subscribe("joint_states", 1, &LocalizationWatchdog::jointStateCallback, this);
  sub_odometry = nh_.subscribe("localization/odometry/sensor_fusion", 1, &LocalizationWatchdog::odometryCallback, this);

  pub_watchdog = nh_.advertise<localization_watchdog::WatchdogStatus>("localization/watchdog", 1);

  clt_enable_driving = nh_.serviceClient<driving_control::EnableDriving>("driving/enable");
}

void LocalizationWatchdog::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                   msg->orientation.w);

  tf::Matrix3x3 m(q);

  m.getRPY(roll_curr_, pitch_curr_, yaw_curr_);

  double int_ax_, int_ay_, int_az_;

  if (imu_msgs_.size() > SET_SIZE - 1)
  {
    imu_msgs_.erase(imu_msgs_.begin());
    imu_msgs_.push_back(*msg);
    int i = 0;
    for (auto &imu_msg : imu_msgs_)
    {
      if (i == 0 || i == SET_SIZE - 1)
      {
        int_ax_ += 0.5 * imu_msg.linear_acceleration.x;
        int_ay_ += 0.5 * imu_msg.linear_acceleration.y;
        int_az_ += 0.5 * (imu_msg.linear_acceleration.z - MOON_GRAVITY);
      }
      else
      {
        int_ax_ += imu_msg.linear_acceleration.x;
        int_ay_ += imu_msg.linear_acceleration.y;
        int_az_ += imu_msg.linear_acceleration.z - MOON_GRAVITY;
      }
      avg_p_ += imu_msg.angular_velocity.x;
      avg_q_ += imu_msg.angular_velocity.y;
      avg_r_ += imu_msg.angular_velocity.z;
      i++;
    }

    if (batch_counter > SET_SIZE - 1)
    {
      imu_vx_ = imu_vx_ + int_ax_ * imu_dt_;
      imu_vy_ = imu_vy_ + int_ay_ * imu_dt_;
      imu_vz_ = imu_vz_ + int_az_ * imu_dt_;
      batch_counter = 0;
    }

    avg_p_ = avg_p_ / (double)SET_SIZE;
    avg_q_ = avg_q_ / (double)SET_SIZE;
    avg_r_ = avg_r_ / (double)SET_SIZE;
  }
  else
  {
    imu_msgs_.push_back(*msg);
  }
  batch_counter++;
}

void LocalizationWatchdog::voCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  vo_msg_ = *msg;
  vo_vx_ = msg->twist.twist.linear.x;
  vo_vy_ = msg->twist.twist.linear.y;
  vo_wz_ = msg->twist.twist.angular.z;
  flag_vo_vx_ = (fabs(vo_vx_) > 0.010);
  flag_vo_vy_ = (fabs(vo_vy_) > 0.010);
  flag_vo_wz_ = (fabs(vo_wz_) > 0.010);
}

void LocalizationWatchdog::woCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  wo_msg_ = *msg;
  wo_vx_ = msg->twist.twist.linear.x;
  wo_vy_ = msg->twist.twist.linear.y;
  wo_wz_ = msg->twist.twist.angular.z;
  flag_wo_vx_ = (fabs(wo_vx_) > 0.010);
  flag_wo_vy_ = (fabs(wo_vy_) > 0.010);
  flag_wo_wz_ = (fabs(wo_wz_) > 0.010);
}

void LocalizationWatchdog::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd_vel_msg_ = *msg;
  cmd_vx_ = msg->linear.x;
  cmd_vy_ = msg->linear.y;
  cmd_wz_ = msg->angular.z;
  flag_cmd_vx_ = (fabs(cmd_vx_) > 0.001);
  flag_cmd_vy_ = (fabs(cmd_vy_) > 0.001);
  flag_cmd_wz_ = (fabs(cmd_wz_) > 0.001);
}

void LocalizationWatchdog::cmdSteeringCallback(const motion_control::SteeringGroup::ConstPtr &msg)
{
  cmd_steer_msg_ = *msg;
  fl_steering_cmd_ = msg->s1;
  bl_steering_cmd_ = msg->s2;
  fr_steering_cmd_ = msg->s3;
  br_steering_cmd_ = msg->s4;
}

void LocalizationWatchdog::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  joint_state_msg_ = *msg;

  int fl_steering_joint_idx;
  int bl_steering_joint_idx;
  int fr_steering_joint_idx;
  int br_steering_joint_idx;
  int fl_wheel_joint_idx;
  int bl_wheel_joint_idx;
  int fr_wheel_joint_idx;
  int br_wheel_joint_idx;

  for (int i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == "fl_steering_arm_tibia_joint")
    {
      fl_steering_joint_idx = i;
    }
    if (msg->name[i] == "bl_steering_arm_tibia_joint")
    {
      bl_steering_joint_idx = i;
    }
    if (msg->name[i] == "fr_steering_arm_tibia_joint")
    {
      fr_steering_joint_idx = i;
    }
    if (msg->name[i] == "br_steering_arm_tibia_joint")
    {
      br_steering_joint_idx = i;
    }
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

  fl_wheel_vels_curr_ = msg->velocity[fl_wheel_joint_idx];
  bl_wheel_vels_curr_ = msg->velocity[bl_wheel_joint_idx];
  fr_wheel_vels_curr_ = msg->velocity[fr_wheel_joint_idx];
  br_wheel_vels_curr_ = msg->velocity[br_wheel_joint_idx];

  fl_steering_curr_ = msg->position[fl_steering_joint_idx];
  bl_steering_curr_ = msg->position[bl_steering_joint_idx];
  fr_steering_curr_ = msg->position[fr_steering_joint_idx];
  br_steering_curr_ = msg->position[br_steering_joint_idx];

  if (fabs(fl_wheel_vels_curr_)<0.005 &&
  fabs(bl_wheel_vels_curr_)<0.005 &&
  fabs(fr_wheel_vels_curr_)<0.005 &&
  fabs(br_wheel_vels_curr_)<0.005)
  {
    imu_vx_ = 0.0;
    imu_vy_ = 0.0;
    imu_vz_ = 0.0;
  }
}

void LocalizationWatchdog::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  odom_msg_ = *msg;
}

void LocalizationWatchdog::WatchdogPublisher()
{
  watchdog_msg_.wasted.data = false;
  watchdog_msg_.immobile.data = false;

  // ------------------------ FLIP DETECTION -----------------------------
  // Check if the robots are flipped
  if (fabs(roll_curr_) > M_PI_2 || fabs(pitch_curr_) > M_PI_2)
  {
    watchdog_msg_.wasted.data = true;
  }

  // --------------------- IMMOBILITY DETECTION --------------------------

  // WO has best indicator when not slipping
  double slip_wo_vo_x = fabs((wo_vx_ - vo_vx_) / wo_vx_) > 1.0 ? 1.0 : fabs((wo_vx_ - vo_vx_) / wo_vx_);
  double slip_wo_vo_y = fabs((wo_vy_ - vo_vy_) / wo_vy_) > 1.0 ? 1.0 : fabs((wo_vy_ - vo_vy_) / wo_vy_);

  // WO vs. cmd vel indicates something is wrong
  double slip_wo_cmd_x = (fabs(cmd_vx_) > 0.01) ? std::min(fabs((wo_vx_ - cmd_vx_) / cmd_vx_),1.0) : 0.0;
  double slip_wo_cmd_y = (fabs(cmd_vy_) > 0.01) ? std::min(fabs((wo_vy_ - cmd_vy_) / cmd_vy_),1.0) : 0.0;

  // VO has most reliable indicator
  double slip_vo_cmd_x = (fabs(cmd_vx_) > 0.01)? std::min(fabs((vo_vx_ - cmd_vx_) / cmd_vx_),1.0) : 0.0;
  double slip_vo_cmd_y = (fabs(cmd_vy_) > 0.01)? std::min(fabs((vo_vy_ - cmd_vy_) / cmd_vy_),1.0) : 0.0;

  // IMU has the best indicator for rotational speed
  double slip_imu_cmd_r = (fabs(cmd_wz_) > 0.01)? std::min(fabs((avg_r_ - cmd_wz_) / cmd_wz_),1.0) : 0.0;

  // IMU can good give info in small time steps
  double slip_imu_cmd_x = 0.0;

  // IMU can good give info in small time steps
  double slip_imu_cmd_y = 0.0;

  // Back wheels are compared cmd vs. current angle
  double slip_b_steer_cmd = (fabs(bl_steering_curr_ - bl_steering_cmd_)
                            + fabs(br_steering_curr_ - br_steering_cmd_))/ (2.0 * M_PI);

  // Front wheels are compared cmd vs. current angle
  double slip_f_steer_cmd = (fabs(fl_steering_curr_ - fl_steering_cmd_)
                            + fabs(fr_steering_curr_ - fr_steering_cmd_))/ (2.0 * M_PI);

  double indicator = 0.0;
  int set_size = 500;
  double temp = 0.0;
  if (indicator_vector.size() > set_size - 1)
  {
    indicator_vector.erase(indicator_vector.begin());

    if(flag_cmd_vx_ && !flag_vo_vy_ && !flag_vo_vx_ && (fabs(wo_vy_)>fabs(vo_vx_))) // cagri's magical condition
    {
      temp = 1.0;
    }
    else if (flag_cmd_vx_ && !flag_vo_vy_ && !flag_vo_vx_)
    {
      temp = 1.0;
    }
    else
    {
      temp = 0.0;
    }
    
    indicator_vector.push_back(temp);
    for (auto &i : indicator_vector)
    {
      indicator += i;
    } 
  }
  else
  {
    indicator_vector.push_back(0.0);
  }

  indicator = indicator / (double) set_size;

  if(indicator > immobility_threshold)
  {
    watchdog_msg_.immobile.data = true;
  }
  
  watchdog_msg_.slip_wo_vo_x = slip_wo_vo_x;
  watchdog_msg_.slip_wo_vo_y = slip_wo_vo_y;
  watchdog_msg_.slip_wo_cmd_x = slip_wo_cmd_x;
  watchdog_msg_.slip_wo_cmd_y = slip_wo_cmd_y;
  watchdog_msg_.slip_vo_cmd_x = slip_vo_cmd_x;
  watchdog_msg_.slip_vo_cmd_y = slip_vo_cmd_y;
  watchdog_msg_.slip_imu_cmd_x = slip_imu_cmd_x;
  watchdog_msg_.slip_imu_cmd_y = slip_imu_cmd_y;
  watchdog_msg_.slip_imu_cmd_r = slip_imu_cmd_r;
  watchdog_msg_.slip_b_steer_cmd = slip_b_steer_cmd;
  watchdog_msg_.slip_f_steer_cmd = slip_f_steer_cmd;
  watchdog_msg_.indicator = indicator;

  // ROS_INFO_STREAM("[" << robot_name_ << "] WATCHDOG. Immobility indicator: " << indicator);
  // -----------------------------------------------------------------------

  if (watchdog_msg_.wasted.data)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "[" << robot_name_ << "] WATCHDOG. I'M WASTED!");
  }
  if (watchdog_msg_.immobile.data)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "[" << robot_name_ << "] WATCHDOG. I'M IMMOBILE!");
  }
  pub_watchdog.publish(watchdog_msg_);
}

void LocalizationWatchdog::ToggleDriving(bool enable)
{
  // Update SF with True Pose
  driving_control::EnableDriving srv_enable_driving;
  srv_enable_driving.request.enable = enable;
  if (clt_enable_driving.call(srv_enable_driving))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] WATCHDOG. Called service TruePose");
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] WATCHDOG. Failed to call Pose Update service");
  }
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