#include "sensor_fusion/sensor_fusion.h"

SensorFusion::SensorFusion(ros::NodeHandle &nh) : nh_(nh) {

  std::string node_name = "sensor_fusion";
  // std::string robot_name;

  if (ros::param::get("robot_name", robot_name) == false) {
    ROS_FATAL("No parameter 'robot_name' specified");
    ros::shutdown();
    exit(1);
  }


  if (ros::param::get(node_name + "/odometry_frame_id", odometry_frame_id) ==
      false) {
    ROS_FATAL("No parameter 'odometry_frame_id' specified");
    ros::shutdown();
    exit(1);
  }
  odometry_frame_id = robot_name + odometry_frame_id;

  if (ros::param::get(node_name + "/odometry_child_frame_id",
                      odometry_child_frame_id) == false) {
    ROS_FATAL("No parameter 'odometry_child_frame_id' specified");
    ros::shutdown();
    exit(1);
  }
  odometry_child_frame_id = robot_name + odometry_child_frame_id;

  if (ros::param::get(node_name + "/position_update_topic",
                      position_update_topic) == false) {
    ROS_FATAL("No parameter 'position_update_topic' specified");
    ros::shutdown();
    exit(1);
  }
  int position_parm_count =0;
  std::string test = node_name +"/"+ robot_name + "/x";
  ROS_ERROR_STREAM("NAME ---->" << test);
  if (ros::param::get(node_name +"/"+ robot_name + "/x" , init_x) ==
      false) {
    ROS_FATAL("No parameter '<robot_name>/x' specified");
    ros::shutdown();
    exit(1);
    position_parm_count = position_parm_count +1;
  }
  else
  {
  position_parm_count = position_parm_count +1;
  }

  if (ros::param::get(node_name +"/"+ robot_name + "/y" , init_y) ==
      false) {
    ROS_FATAL("No parameter '<robot_name>/x' specified");
    ros::shutdown();
    exit(1);

  }
  else
  {
  position_parm_count = position_parm_count +1;
  }

  src2GetTruePoseClient_ =
      nh_.serviceClient<srcp2_msgs::LocalizationSrv>("get_true_pose");
  getTruePoseServer_ = nh_.advertiseService(
      "true_pose", &SensorFusion::getTruePoseFromSRC2_, this);
  // clt_restart_kimera_ =
  // nh.serviceClient<std_srvs::Trigger>("/kimera_vio_ros/kimera_vio_ros_node/restart_kimera_vio");

  averageIMU_ = false; // if true, IMU attitude will be averaged between wheel
                       // odom updates; if false latest IMU attitude is used
  firstVO_ = true;
  firstWO_ = true;
  firstIMU_ = true;
  init_true_pose_ = false;
  incCounter_ = 0;
  rollInc_ = 0;
  pitchInc_ = 0;
  yawInc_ = 0;
  mobility_.data = MOBILE;
  averageAccel_ = true;
  accelCount_ = 0;
  start_time_dist = ros::Time::now();

  // initialized_=NOT_INITIALIZED;

  subVO_ = nh_.subscribe("vo", 1, &SensorFusion::voCallback_, this);

  subImu_ = nh_.subscribe("imu_filtered", 10, &SensorFusion::imuCallback_,
                          this); // Robot namespace here

  subWheelOdom_ = nh_.subscribe("dead_reckoning/odometry", 1,
                                &SensorFusion::wheelOdomCallback_, this);

  subDrivingMode_ = nh_.subscribe("driving/driving_mode", 1,
                                  &SensorFusion::drivingModeCallback_, this);
  subPositionUpdate_ = nh_.subscribe(

      position_update_topic, 1, &SensorFusion::positionUpdateCallback_, this);

  subPositionUpdate_ = nh_.subscribe(
      "/initial_attitude", 1, &SensorFusion::attitudeInitCallback_, this);

  pubOdom_ = nh_.advertise<nav_msgs::Odometry>(
      "localization/odometry/sensor_fusion", 1);

  pubStatus_ = nh_.advertise<std_msgs::Int64>(
      "state_machine/localized_base", 100);

  pubMobility_ = nh_.advertise<std_msgs::Int64>(
      "state_machine/mobility" , 1);

  pubSlip_ = nh_.advertise<geometry_msgs::PointStamped>(
      "localization/odometry/slip", 1);

  pubInitAttitude_ = nh_.advertise<geometry_msgs::Quaternion>(
          "/initial_attitude", 1);

  g_(0, 0) = 0.0;
  g_(1, 0) = 0.0;
  g_(2, 0) = -1.62;
  accelIMU_(0, 0) = 0.0;
  accelIMU_(1, 0) = 0.0;
  accelIMU_(2, 0) = 0.0;
  double sigVel = .1;
  double sigPos = .01;
  Q_ << pow(sigPos, 2), 0, 0, 0, 0, 0, 0, pow(sigPos, 2), 0, 0, 0, 0, 0, 0,
      pow(sigPos, 2), 0, 0, 0, 0, 0, 0, pow(sigVel, 2), 0, 0, 0, 0, 0, 0,
      pow(sigVel, 2), 0, 0, 0, 0, 0, 0, pow(sigVel, 2);

  P_ = Q_;
  driving_mode_ = 0;

  Hodom_ << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  Hposition_ << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  double sigPosition = 1e-4;
  double sigConstrain = .1;
  Rposition_ << pow(sigPosition, 2), 0, 0, 0, 0, 0, 0, pow(sigPosition, 2), 0,
      0, 0, 0, 0, 0, pow(sigConstrain, 2), 0, 0, 0, 0, 0, 0,
      pow(sigConstrain, 2), 0, 0, 0, 0, 0, 0, pow(sigConstrain, 2), 0, 0, 0, 0,
      0, 0, pow(sigConstrain, 2);

  double slip_ = 0;
  double sigWO = .05;
  double sigVO = .25;
  Rwo_ << pow(sigWO, 2), 0, 0, 0, pow(sigWO, 2), 0, 0, 0, pow(sigWO, 2);

  Rvo_ << pow(sigVO, 2), 0, 0, 0, pow(sigVO, 2), 0, 0, 0, pow(sigVO, 2);

  F_ << 1, 0, 0, .2, 0, 0, 0, 1, 0, 0, .2, 0, 0, 0, 1, 0, 0, .2, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  x_ << 0, 0, 0, 0, 0, 0;
  if(  position_parm_count == 2)
  {
    x_(0, 0) = init_x;
    x_(1, 0) = init_y;
    x_(2, 0) = 1.65;
    x_(3, 0) = 0.0;
    x_(4, 0) = 0.0;
    x_(5, 0) = 0.0;
    init_true_position_ = true;

    ROS_ERROR_STREAM("SENSOR FUSION " << robot_name << " Initial X " << init_x << " Initial Y " << init_y);

  }


  last_x_=x_;

}
void SensorFusion::PublishInitAttitude()
{
        pubInitAttitude_.publish(q_msg);
}
void SensorFusion::attitudeInitCallback_( const geometry_msgs::Quaternion::ConstPtr &msg){
  // if this robot already got true attitude from SRC2.  Do nothing. Just return
  if(true_pose_from_src2 || init_true_pose_ )
  {
      return;
  }
  init_true_attitude_ = true;
  if(init_true_position_ && init_true_attitude_)
  {
    init_true_pose_=true;
  }
  tf::Quaternion q(msg->x, msg->y, msg->z,
                   msg->w);
  tf::Matrix3x3 R_init_true_b_n(q);
  R_imu_nav_o_ = R_init_true_b_n * R_body_imu_.transpose();

}
void SensorFusion::imuCallback_(const sensor_msgs::Imu::ConstPtr &msg) {
  // std::cout << " IMU Callback " << std::endl;
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                   msg->orientation.w);

  tf::Matrix3x3 m(q);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  if (firstIMU_)
    R_body_imu_ = m;

  if (averageIMU_) {

    rollInc_ = rollInc_ + roll;
    pitchInc_ = pitchInc_ + pitch;
    incCounter_ = incCounter_ + 1.0;
    if (incCounter_ > 1.0) {
      // if there has been a roll over
      double yawAvg = (yawInc_ / (incCounter_ - 1.0));
      if (fabs(yaw - yawAvg) > 3.14) {
        std::cout << "Roll Over Detected " << yaw << " " << yawAvg << std::endl;
        if (yaw < yawAvg)
          yaw = yaw + 6.2831185;
        else
          yaw = yaw - 6.2831185;
      }
    }

    yawInc_ = yawInc_ + yaw;
  } else {
    // not averaging, just using current roll, pitch, yaw
    rollInc_ = roll;
    pitchInc_ = pitch;
    yawInc_ = yaw;
  }

  firstIMU_ = false;

  if (firstWO_)
    R_imu_nav_o_ = R_body_imu_.transpose();
  if (averageAccel_ && !std::isnan(msg->linear_acceleration.x )
    && !std::isnan(msg->linear_acceleration.y ) &&
  !std::isnan(msg->linear_acceleration.z ) ) {
    //			ROS_ERROR_STREAM("Accel IMU " << msg->linear_acceleration.x << " "
    //<< msg->linear_acceleration.y << " " << msg->linear_acceleration.z);
    accelIMU_(0, 0) = msg->linear_acceleration.x + accelIMU_(0, 0);
    accelIMU_(1, 0) = msg->linear_acceleration.y + accelIMU_(1, 0);
    accelIMU_(2, 0) = msg->linear_acceleration.z + accelIMU_(2, 0);
    accelCount_ = accelCount_ + 1;
  } else {
    accelIMU_(0, 0) = msg->linear_acceleration.x;
    accelIMU_(1, 0) = msg->linear_acceleration.y;
    accelIMU_(2, 0) = msg->linear_acceleration.z;
  }
  //	ROS_INFO("RPY %f  %f
  //%f\n",roll*180.0/3.14,pitch*180.0/3.14,yaw*180.0/3.14);
}

bool SensorFusion::getTruePoseFromSRC2_(
    sensor_fusion::GetTruePose::Request &req,
    sensor_fusion::GetTruePose::Response &res) {

  ROS_INFO(" Calling the SRC2 Get True Pose Service ");
  true_pose_from_src2 = true;
  srcp2_msgs::LocalizationSrv srv;
  if (req.start) {
    srv.request.call = true;

    if (src2GetTruePoseClient_.call(srv)) {
      pose_ = srv.response.pose;

      tf::Quaternion q(pose_.orientation.x, pose_.orientation.y,
                       pose_.orientation.z, pose_.orientation.w);

      // if called to initialize, call topic to initialize others
      if(req.initialize)
      {
          q_msg.x = q.x();
          q_msg.y = q.y();
          q_msg.z = q.z();
          q_msg.w = q.w();
          pubInitAttitude_.publish(q_msg);
          have_init_attitude =true;
          publish_attitude = true;
      }

      tf::Matrix3x3 R_init_true_b_n(q);
      R_imu_nav_o_ = R_init_true_b_n * R_body_imu_.transpose();
      x_(0, 0) = pose_.position.x;
      x_(1, 0) = pose_.position.y;
      x_(2, 0) = pose_.position.z;
      x_(3, 0) = 0.0;
      x_(4, 0) = 0.0;
      x_(5, 0) = 0.0;
      init_pos_z_= pose_.position.z;
      // also re-init P
      P_ = Q_;
      P_(0, 0) = 1e-3;
      P_(1, 1) = 1e-3;
      P_(2, 2) = 1e-3;
      P_(3, 3) = 1e-1;
      P_(4, 4) = 1e-1;
      P_(5, 5) = 1e-1;
      init_true_pose_ = true;
      res.success = true;

      return true;
    } else {
      ROS_ERROR(" SRC2 Get True Pose Service Failed ");
      res.success = false;
      return false;
    }
  }
  return false;
}

void SensorFusion::drivingModeCallback_(const std_msgs::Int64::ConstPtr &msg) {
  driving_mode_ = msg->data;

  // Stop
  // if(driving_mode_==4) {
  //         Q_(0,0)=0.0;
  //         Q_(1,1)=0.0;
  //         Q_(2,2)=0.0;
  //         Q_(3,3)=0.0;
  //         Q_(4,4)=0.0;
  //         Q_(5,5)=0.0;
  // }
  // if(driving_mode_==0) {
  //         Q_(0,0)=pow(0.01,2);
  //         Q_(1,1)=pow(0.01,2);
  //         Q_(2,2)=pow(0.05,2);
  //         Q_(3,3)=pow(0.1,2);
  //         Q_(4,4)=pow(0.01,2);
  //         Q_(5,5)=pow(0.01,2);
  // }
  // //crab
  // else if(driving_mode_ == 1) {
  //
  //         Q_(0,0)=pow(0.01,2);
  //         Q_(1,1)=pow(0.01,2);
  //         Q_(2,2)=pow(0.05,2);
  //         Q_(3,3)=pow(0.1,2);
  //         Q_(4,4)=pow(0.1,2);
  //         Q_(5,5)=pow(0.01,2);
  // }
  // // DACK
  // else if(driving_mode_== 2) {
  //
  //         Q_(0,0)=pow(0.01,2);
  //         Q_(1,1)=pow(0.01,2);
  //         Q_(2,2)=pow(0.05,2);
  //         Q_(3,3)=pow(0.1,2);
  //         Q_(4,4)=pow(0.01,2);
  //         Q_(5,5)=pow(0.01,2);
  // }
  // // turn in place
  // else if(driving_mode_== 3) {
  //
  //         Q_(0,0)=pow(0.0,2);
  //         Q_(1,1)=pow(0.0,2);
  //         Q_(2,2)=pow(0.0,2);
  //         Q_(3,3)=pow(0.0,2);
  //         Q_(4,4)=pow(0.0,2);
  //         Q_(5,5)=pow(0.1,2);
  // }
  // else if(driving_mode_==4) {
  //         Q_(0,0)=0.0;
  //         Q_(1,1)=0.0;
  //         Q_(2,2)=0.0;
  //         Q_(3,3)=0.0;
  //         Q_(4,4)=0.0;
  //         Q_(5,5)=0.0;
  // }
  // else if (driving_mode_ == 0)
  // {
  //  Q_(0,0)=0.0;
  //  Q_(1,1)=0.0;
  //  Q_(2,2)=0.0;
  //  Q_(3,3)=pow(0.1,2);
  //  Q_(4,4)=pow(0.1,2);
  //  Q_(5,5)=pow(0.01,2);
  //
}

void SensorFusion::wheelOdomCallback_(const nav_msgs::Odometry::ConstPtr &msg) {

  if (firstWO_) {
    if (!firstIMU_)
      firstWO_ = false;
    // since kimera is init with true pose, pick up true global attitude on
    // first call
    // later we can get this directly from get true pose

    tf::Quaternion q(0.0, 0.0, 0.0, 1.0);

    tf::Matrix3x3 R_init_b_n(q);
    R_imu_nav_o_ = R_init_b_n * R_body_imu_.transpose();

    pose_ = msg->pose.pose;

    // rotate kimeta body axis velocity into the nav frame
    tf::Vector3 vn_imu;
    tf::Vector3 vb_wo(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z);

    Rbn_ = R_imu_nav_o_ * R_body_imu_;
    vn_imu = Rbn_ * vb_wo;

    x_(3,0)= vn_imu.x();
    x_(4,0)= vn_imu.y();
    x_(5,0)= vn_imu.z();
  }

  // std::cout <<"Wheel Odom Callback " << std::endl;
  if (averageIMU_) {
    R_body_imu_.setRPY(rollInc_ / incCounter_, pitchInc_ / incCounter_,
                       yawInc_ / incCounter_);
    rollInc_ = 0.0;
    pitchInc_ = 0.0;
    yawInc_ = 0.0;
    incCounter_ = 0.0;
  } else {
    // not averaging, just use current roll, pitch & yaw
    R_body_imu_.setRPY(rollInc_, pitchInc_, yawInc_);
  }

  Eigen::MatrixXd accel(3, 1);
  if (averageAccel_ && accelCount_>0) {
    // ROS_ERROR_STREAM("Accel IMU " << accelIMU_.transpose());
    // ROS_ERROR_STREAM("Accel IMU Count " << accelCount_);

    accel(0, 0) = accelIMU_(0, 0) / accelCount_;
    accel(1, 0) = accelIMU_(1, 0) / accelCount_;
    accel(2, 0) = accelIMU_(2, 0) / accelCount_;
    //			ROS_ERROR_STREAM(" Accel Avg" << accel.transpose());
    accelIMU_(0, 0) = 0.0;
    accelIMU_(1, 0) = 0.0;
    accelIMU_(2, 0) = 0.0;
    accelCount_ = 0;
  } else {
    accel(0, 0) = accelIMU_(0, 0);
    accel(1, 0) = accelIMU_(1, 0);
    accel(2, 0) = accelIMU_(2, 0);
  }

  Rbn_ = R_imu_nav_o_ * R_body_imu_;

  double dt = msg->header.stamp.toSec() - lastTime_wo_.toSec();

  // state predicition
  F_(0, 3) = dt;
  F_(1, 4) = dt;
  F_(2, 5) = dt;
  Q_(0, 0) = pow(0.05 * dt * dt, 2);
  Q_(1, 1) = pow(0.05 * dt * dt, 2);
  Q_(2, 2) = pow(0.05 * dt * dt, 2);
  Q_(3, 3) = pow(0.05 * dt, 2);
  Q_(4, 4) = pow(0.05 * dt, 2);
  Q_(5, 5) = pow(0.05 * dt, 2);

  Eigen::MatrixXd RBN(3, 3);
  tf::Vector3 row;
  row = Rbn_.getRow(0);
  RBN(0, 0) = row.x();
  RBN(0, 1) = row.y();
  RBN(0, 2) = row.z();
  row = Rbn_.getRow(1);
  RBN(1, 0) = row.x();
  RBN(1, 1) = row.y();
  RBN(1, 2) = row.z();
  row = Rbn_.getRow(2);
  RBN(2, 0) = row.x();
  RBN(2, 1) = row.y();
  RBN(2, 2) = row.z();
  //	x_ = F_*x_;
  x_(0, 0) = x_(0, 0) + dt * x_(3, 0);
  x_(1, 0) = x_(1, 0) + dt * x_(4, 0);
  x_(2, 0) = x_(2, 0) + dt * x_(5, 0);
  vAccNav_ = RBN * accel + g_;
  x_(3, 0) = x_(3, 0) + dt * vAccNav_(0, 0);
  x_(4, 0) = x_(4, 0) + dt * vAccNav_(1, 0);
  x_(5, 0) = x_(5, 0) + dt * vAccNav_(2, 0);

  Eigen::MatrixXd G(6, 6);
  G.setZero();
  G(0, 0) = 1.0;
  G(1, 1) = 1.0;
  G(2, 2) = 1.0;

  row = Rbn_.getRow(0);

  G(3, 3) = row.x();
  G(3, 4) = row.y();
  G(3, 5) = row.z();
  row = Rbn_.getRow(1);
  G(4, 3) = row.x();
  G(4, 4) = row.y();
  G(4, 5) = row.z();
  row = Rbn_.getRow(2);
  G(5, 3) = row.x();
  G(5, 4) = row.y();
  G(5, 5) = row.z();

  P_ = F_ * P_ * F_.transpose() + G * Q_ * G.transpose();

  double roll, pitch, yaw;
  Rbn_.getRPY(roll, pitch, yaw);
  if ((pitch * 180 / 3.1414926) > 60) {
    ROS_WARN_THROTTLE(10, "Skipping Wheel Odom Update Pitch: %f",
                      pitch * 180 / 3.1414926);
    ROS_WARN_STREAM_THROTTLE(10, " WO Vel " << vb_wo_.x());
    ROS_WARN_STREAM_THROTTLE(10, " VO Vel " << vb_vo_.x());

  } else {

    if ((pitch * 180 / 3.1414926) < -15) {
      ROS_WARN_THROTTLE(10, "Robot Climbing Up! Pitch: %f",
                        pitch * 180 / 3.1414926);
      ROS_WARN_STREAM_THROTTLE(10, " WO Vel " << vb_wo_.x());
      ROS_WARN_STREAM_THROTTLE(10, " VO Vel " << vb_vo_.x());
      if ((pitch * 180 / 3.1414926) < -35) {
        ROS_ERROR("Robot Cant Climb! Pitch: %f",
                           pitch * 180 / 3.1414926);
                           ROS_ERROR("SENDING IMMOBILITY FLAG TO MOBILE CHEcKER");
         mobility_.data = IMMOBILE;
         pubMobility_.publish(mobility_);
      }
    }

    if ((pitch * 180 / 3.1414926) > 15) {
      ROS_WARN_THROTTLE(10, "Robot Climbing Down! Pitch: %f",
                        pitch * 180 / 3.1414926);
      ROS_WARN_STREAM_THROTTLE(10, " WO Vel " << vb_wo_.x());
      ROS_WARN_STREAM_THROTTLE(10, " VO Vel " << vb_vo_.x());
    }

    tf::Vector3 vb_wo(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z);

    vb_wo_ = vb_wo;

    // rotate kimeta body axis velocity into the nav frame
    tf::Vector3 vn_wo;
    vn_wo = Rbn_ * vb_wo;

    // if(v_body_wo_.distance(v_body_vo_)> .6 && v_body_wo_.length() > .4 &&
    // v_body_vo_.length() < .4 ){
    //   ROS_ERROR("Wheel Slip Detected in Sensor Fusion");
    //   return;
    // }

    // if(slip_<.9 && driving_mode_!=0) {

    // kimera measurement update
    zWO_(0, 0) = vn_wo.x();
    zWO_(1, 0) = vn_wo.y();
    zWO_(2, 0) = vn_wo.z();

    Eigen::Vector3d Innovation;
    Innovation = zWO_ - Hodom_ * x_;

    Eigen::Vector3d Hx = Hodom_ * x_;

    // if ((Innovation.norm() > 1.0 && init_true_pose_) || std::isnan(Innovation.norm()))

    if (std::isnan(Innovation.norm())) {
ROS_ERROR_STREAM("Skipping WO:  Failed Innovation Check "
                       << Innovation.norm());
      // ROS_INFO_STREAM(" SF: WO Hx " << Hx.transpose()  );
      // ROS_INFO_STREAM(" WO" << vn_wo.x() << " " << vn_wo.y() << "  "<<
      // vn_wo.z()); ROS_INFO_STREAM(" Innov " << Innovation.transpose());
      lastTime_wo_ = msg->header.stamp;
      publishOdom_();
      return;
    }
    Eigen::MatrixXd S(3, 3);
    S = Rwo_ + Hodom_ * P_ * Hodom_.transpose();

    Eigen::MatrixXd K(6, 3);
    K = (P_ * Hodom_.transpose()) * S.inverse();
    Eigen::MatrixXd I(6, 6);
    I.setIdentity();
    P_ = (I - K * Hodom_) * P_;
    Eigen::MatrixXd xPred(2, 1);
    xPred << x_(0, 0), x_(1, 0);

    x_ = x_ + K * (zWO_ - Hodom_ * x_);

    //	if(pow(pow(x_(0,0)-xPred(0,0),2)+pow(x_(1,0)-xPred(1,0),2),.5)

    // }else{
    //
    //   if(slip_>.9) ROS_ERROR(" Skip WO Due to High Slip");
    // }
  }

  lastTime_wo_ = msg->header.stamp;
  publishOdom_();

  if ((fabs(lastTime_wo_.toSec() - lastTime_vo_.toSec()) > 30) && !firstVO_) {
    ROS_INFO_STREAM(" VO NODE FAIL!? ");
    ROS_INFO_STREAM(" lastTime_wo " << lastTime_wo_.toSec());
    ROS_INFO_STREAM(" lastTime_vio " << lastTime_vo_.toSec());
    ROS_INFO_STREAM(" dt" << fabs(lastTime_wo_.toSec() - lastTime_vo_.toSec()));
    std_srvs::Trigger trig;
    clt_restart_kimera_.call(trig);
    lastTime_vo_ = msg->header.stamp;
  }
}

void SensorFusion::positionUpdateCallback_(
    const geometry_msgs::Pose::ConstPtr &msg) {

  ROS_WARN("Homing Update in Sensor Fusion");

  // handle the excavtor position update.
  double scalar=1;
  if(msg->orientation.x){
    ROS_WARN("Scaling the Position Error Covariance By Dumping Mass");
    scalar=(50.0*50.0)/(msg->orientation.x*msg->orientation.x);
  }

  zPosition_(0, 0) = x_[0] + msg->position.x;
  zPosition_(1, 0) = x_[1] + msg->position.y;
  zPosition_(2, 0) = init_pos_z_;
  // zPosition_(2, 0) = 0.0;
  zPosition_(3, 0) = 0.0;
  zPosition_(4, 0) = 0.0;
  zPosition_(5, 0) = 0.0;
  Eigen::MatrixXd S(6, 6);
  S = scalar*Rposition_ + Hposition_ * P_ * Hposition_.transpose();

  Eigen::MatrixXd K(6, 6);
  K = (P_ * Hposition_.transpose()) * S.inverse();
  Eigen::MatrixXd I(6, 6);
  I.setIdentity();
  P_ = (I - K * Hposition_) * P_;
  x_ = x_ + K * (zPosition_ - Hposition_ * x_);
  homingUpdateFlag_ = true;
  x_diff_=0;
  y_diff_=0;
}

void SensorFusion::voCallback_(const nav_msgs::Odometry::ConstPtr &msg) {

  if (!firstIMU_) {
    firstVO_ = false;
  }

  // std::cout << " WVU Callback " << std::endl;
  // TODO: if pose NaN, then stop the rover, restart kimera with the latest pose
  // (the one before NaN)
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // get kimera velocity represented in body frame

  tf::Vector3 vb_kimera(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);

  // rotate kimera body axis velocity into the nav frame
  tf::Vector3 vn_kim;
  vn_kim = Rbn_ * vb_kimera;

  // kimera measurement update
  zVO_(0, 0) = vn_kim.x();
  zVO_(1, 0) = vn_kim.y();
  zVO_(2, 0) = vn_kim.z();
  Eigen::MatrixXd S(3, 3);
  S = Rvo_ + Hodom_ * P_ * Hodom_.transpose();

  Eigen::MatrixXd K(6, 3);
  K = (P_ * Hodom_.transpose()) * S.inverse();
  Eigen::MatrixXd I(6, 6);
  I.setIdentity();
  Eigen::Vector3d Innovation;

  Innovation = zVO_ - Hodom_ * x_;

  lastTime_vo_ = msg->header.stamp;

  if (Innovation.norm() > .5 || std::isnan(Innovation.norm())) {
    // ROS_INFO_STREAM(" SF: VO update skipped! " );
    return;
  }
  vb_vo_ = vb_kimera;

  P_ = (I - K * Hodom_) * P_;
  x_ = x_ + K * (zVO_ - Hodom_ * x_);

  // tf::Vector3	vn_vo_(x_[3],x_[4],x_[5]);
  // vb_vo_=Rbn_.transpose()*vn_vo_;
}

void SensorFusion::initializationStatus_() {
  // std_msgs::Int64 status;
  if (init_true_pose_) {
    status_.data = INITIALIZED;
  } else {
    status_.data = NOT_INITIALIZED;
  }

  pubStatus_.publish(status_);
}

void SensorFusion::publishOdom_() {
  nav_msgs::Odometry updatedOdom;
  geometry_msgs::PointStamped slip;

  updatedOdom.header.stamp = lastTime_wo_;
  updatedOdom.header.frame_id = odometry_frame_id;
  updatedOdom.child_frame_id = odometry_child_frame_id;

  tf::Quaternion qup;
  qup.normalize();
  Rbn_.getRotation(qup);

  double roll, pitch, yaw;
  Rbn_.getRPY(roll, pitch, yaw);
  // ROS_INFO("RPY in WO %f  %f
  // %f\n",roll*180.0/3.14,pitch*180.0/3.14,yaw*180.0/3.14);
  //  ROS_INFO_STREAM(" state x: " << x_.transpose() );

  // slip check

  if (vb_wo_.length() != 0.0 && status_.data == INITIALIZED &&
      vb_vo_.length() < .2 && vb_wo_.length() > .1 && driving_mode_ == 2 && init_true_pose_== true) {
    mobility_.data = MOBILE; // TODO: It might be here too! If it is immobile at
                             // the previous step, it sends immobile again.


    if (slipTimer == 0)
    {
      slipTimer = ros::Time::now().toSec();
      temp_x_ = x_(0);
      temp_y_ = x_(1);
    }
    else
    {
      if (ros::Time::now().toSec() - slipTimer < 25 && slipCount_ > 40 && ros::Time::now().toSec() - slipTimer > 2)
      {
        ROS_ERROR_STREAM_THROTTLE(5,"Frequent Slip: Slip Count: " << slipCount_);
        ROS_ERROR_STREAM_THROTTLE(5, "Delta Time: " << ros::Time::now().toSec() - slipTimer);
        // ROS_ERROR_STREAM("DRIVING MODE IN SLIP"<<d
        if (homingUpdateFlag_)
        {
          ROS_ERROR_THROTTLE(5, "Rover performed homing recently, skipping high slip flag");
          homingUpdateFlag_ = false;
        }
        else
        {
          ROS_ERROR_STREAM("Immobility!" << slipCount_);
          ROS_ERROR_STREAM_THROTTLE(5, "Delta Time: " << ros::Time::now().toSec() - slipTimer);
          // slipped_x_ = x_(0);
          // slipped_y_ = x_(1);
          // x_diff_ = slipped_x_ - temp_x_;
          // y_diff_ = slipped_y_ - temp_y_;

          mobility_.data = IMMOBILE;
        }
        slipCount_ = 0;
        slipTimer = 0;
      }
      else
      {
        if (ros::Time::now().toSec() - slipTimer > 25)
        {
          slipTimer = 0;
          slipCount_ = 0;
        }
        mobility_.data = MOBILE;
      }
    }
    slip.point.x = (vb_wo_.x() - vb_vo_.x()) / vb_wo_.x();
    slip.point.y = (vb_wo_.y() - vb_vo_.y()) / vb_wo_.y();
    slip.point.z = 0; //z
    slip.header.stamp = lastTime_wo_;
    slip.header.frame_id = odometry_frame_id;
    // pubSlip_.publish(slip);
    slip_ = fabs(slip.point.x);
    if (slip_ > 0.9 && slip_<1.0)
    {
      slipCount_++;
      ROS_ERROR_STREAM_THROTTLE(5, "High Longitidunal Slip Detected: " << slip_);
    }
    if (fabs(slip.point.y) > 0.9 && fabs(slip.point.y)<1.0)
    {
      ROS_ERROR_STREAM_THROTTLE(5, "High Lateral Slip Detected: " << slip.point.y);
    }
    if (mobility_.data == 0)
    {
      slipped_x_ = x_(0);
      slipped_y_ = x_(1);
      x_diff_ = x_diff_ + slipped_x_ - temp_x_;
      y_diff_ = x_diff_ + slipped_y_ - temp_y_;

      ROS_ERROR_STREAM("[" << robot_name << "] " <<"Rover is possibly stuck!");
      ROS_INFO_STREAM("[" << robot_name << "] " <<"Accumulated x_diff: " << x_diff_);
      ROS_INFO_STREAM("[" << robot_name << "] " <<"Accumulated y_diff: " << y_diff_);
      ROS_INFO_STREAM("[" << robot_name << "] " <<"temp_x: " << temp_x_);
      ROS_INFO_STREAM("[" << robot_name << "] " <<"temp_y: " << temp_y_);
      ROS_INFO_STREAM("[" << robot_name << "] " <<"slipped_x: " << slipped_x_);
      ROS_INFO_STREAM("[" << robot_name << "] " <<"slipped_y: " << slipped_y_);
      ROS_ERROR_STREAM("Frequent Slip Flag, slipCount_= " << slipCount_);
      temp_x_=x_(0);
      temp_y_=x_(1);


      // ROS_ERROR("Sending immobility flag to Mobility Checker");
    }
    // ROS_ERROR_STREAM("IMMOBILITY" << mobility_.data);
    pubMobility_.publish(mobility_);
  }

  if(std::isnan(x_.sum())){
    x_ = last_x_;
    P_= Q_;
  }
  updatedOdom.pose.pose.position.x = x_(0);
  updatedOdom.pose.pose.position.y = x_(1);
  // updatedOdom.pose.pose.position.z = x_(2);
  updatedOdom.pose.pose.position.z = init_pos_z_;

  updatedOdom.pose.pose.orientation.x = qup.x();
  updatedOdom.pose.pose.orientation.y = qup.y();
  updatedOdom.pose.pose.orientation.z = qup.z();
  updatedOdom.pose.pose.orientation.w = qup.w();

  // put body_vel

  tf::Vector3 v_body_ekf;
  tf::Vector3 v_nav_ekf(x_[3], x_[4], x_[5]);
  v_body_ekf = Rbn_.transpose() * v_nav_ekf;
  updatedOdom.twist.twist.linear.x = v_body_ekf.x();
  updatedOdom.twist.twist.linear.y = v_body_ekf.y();
  updatedOdom.twist.twist.linear.z = v_body_ekf.z();

  updatedOdom.pose.covariance[0] = P_(0, 0);
  updatedOdom.pose.covariance[1] = P_(1, 1);

  pubOdom_.publish(updatedOdom);
  pubSlip_.publish(slip);
  pose_ = updatedOdom.pose.pose;

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = updatedOdom.header.stamp;
  odom_trans.header.frame_id = updatedOdom.header.frame_id;
  odom_trans.child_frame_id = updatedOdom.child_frame_id;
  odom_trans.transform.translation.x = updatedOdom.pose.pose.position.x;
  odom_trans.transform.translation.y = updatedOdom.pose.pose.position.y;
  odom_trans.transform.translation.z = updatedOdom.pose.pose.position.z;
  odom_trans.transform.rotation.x = updatedOdom.pose.pose.orientation.x;
  odom_trans.transform.rotation.y = updatedOdom.pose.pose.orientation.y;
  odom_trans.transform.rotation.z = updatedOdom.pose.pose.orientation.z;
  odom_trans.transform.rotation.w = updatedOdom.pose.pose.orientation.w;
  odom_broadcaster_.sendTransform(odom_trans);

  last_x_ =x_;



  ros::Duration distanceTimer(120.0); // Timeout of 20 seconds

  if (ros::Time::now() - start_time_dist < ros::Duration(1.0)) {
    distPrev = sqrt(pow(updatedOdom.pose.pose.position.x, 2) + pow(updatedOdom.pose.pose.position.y, 2));
  } else if (ros::Time::now() -start_time_dist > distanceTimer) {
    distNow = sqrt(pow(updatedOdom.pose.pose.position.x, 2) + pow(updatedOdom.pose.pose.position.y, 2));
    distDiff= fabs(distNow - distPrev);
    // ROS_ERROR_STREAM("Distance changed in 120 seconds, Diff: " << distDiff);
    start_time_dist = ros::Time::now();
  }
// ROS_ERROR_STREAM("DistPrev???: " << distPrev);
// ROS_ERROR_STREAM("DistNow???: " << distNow);
// ROS_ERROR_STREAM("DistDiff???: " << distDiff);


}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_fusion");
  ros::NodeHandle nh("");
  ros::Rate rate(100);
  ROS_INFO(" Sensor Fusion started ");

  SensorFusion localizer(nh);

  // ros::spin();
  int counter =0;
  while (ros::ok()) {

    localizer.initializationStatus_();
    ros::spinOnce();
    if(localizer.have_init_attitude && localizer.publish_attitude && counter ==100)
    {
      localizer.PublishInitAttitude();
      counter = 0;
    }
    rate.sleep();
    counter = counter +1;

  }

  return 0;
}
