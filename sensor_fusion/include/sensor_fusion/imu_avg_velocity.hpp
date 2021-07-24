/*!
 * \imu_avg_velocity.h
 * \brief imu_avg_velocity (...).
 *
 * Averages IMU to obtain averaged vx, vy, vz, r, p, q.
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date June 01, 2020
 */

#ifndef IMU_AVG_VELOCITY_H
#define IMU_AVG_VELOCITY_H


// Include cpp important headers
#include <math.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <termios.h>
#include <vector>
#include <numeric>

// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

class ImuAvgVelocity
{
public:

    ImuAvgVelocity(ros::NodeHandle & nh);

private:

    ros::NodeHandle & nh_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

    ros::Subscriber sub_imu;
    ros::Subscriber sub_joint_states;

    ros::Publisher pub_twist;

    std::vector<sensor_msgs::Imu> imu_msgs_;
    
    double avg_vx_ = 0.0;
    double avg_vy_ = 0.0;
    double avg_vz_ = 0.0;

    const int SET_SIZE = 50;
    const double imu_dt_ = 0.01;

    const double MOON_GRAVITY = 1.625;

    int counter = 0;
 };

#endif //IMU_AVG_VELOCITY_H