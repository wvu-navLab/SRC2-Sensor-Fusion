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

class ImuAvgVelocity
{
public:

    ImuAvgVelocity(ros::NodeHandle & nh);

private:

    ros::NodeHandle & nh_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    ros::Subscriber sub_imu;

    ros::Publisher pub_twist;

    std::vector<sensor_msgs::Imu> imu_msgs_;

    const int SET_SIZE = 50;
    const double imu_dt_ = 0.01;

    const double MOON_GRAVITY = 1.625;
 };

#endif //IMU_AVG_VELOCITY_H