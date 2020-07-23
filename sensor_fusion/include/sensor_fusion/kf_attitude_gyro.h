/*!
 * \kf_attitude_gyro.h
 * \brief kf_attitude_gyro (...).
 *
 * Kalman filter gyro for attitude prediction, low noise hf, and attitude measurements update, noisy  (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date July 14, 2020
 */

#ifndef KF_ATTITUDE_GYRO_H
#define KF_ATTITUDE_GYRO_H


// Include cpp important headers
#include <math.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <termios.h>
#include <vector>
#include <numeric>
#include <eigen3/Eigen/Dense>

// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

class KFAttitudeGyro
{
public:

    KFAttitudeGyro(ros::NodeHandle & nh);

private:

    ros::NodeHandle & nh_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void publishStates(const double & state, const double & covariance, const ros::Time & time, const ros::Publisher & pub);
    void publishBiases(const double & bx, const double & by, const double & bz, const ros::Time & time, const ros::Publisher & pub);

    ros::Subscriber subImu;
    ros::Subscriber subOdom;

    ros::Publisher pubRollMeasured;
    ros::Publisher pubPitchMeasured;
    ros::Publisher pubYawMeasured;
    ros::Publisher pubOdomRoll;
    ros::Publisher pubOdomPitch;
    ros::Publisher pubOdomYaw;

    ros::Publisher pubRollEstimated;
    ros::Publisher pubPitchEstimated;
    ros::Publisher pubYawEstimated;
    ros::Publisher pubBiasesEstimated;

    ros::Publisher pubImuFiltered;    

    Eigen::Matrix <double, 6, 1> x_;
    Eigen::Matrix <double, 6, 6> P_;

    double dt_;
    double last_time_;
    bool firstIMUCallback_;
 };

#endif //KF_ATTITUDE_GYRO_H
