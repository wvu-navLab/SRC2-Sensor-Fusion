#ifndef VIO_ATTITUDE_LOCALIZATION_H
#define VIO_ATTITUDE_LOCALIZATION_H


// Include cpp important headers
#include <math.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <termios.h>
#include <vector>


// ROS headers
#include <ros/ros.h>
#include <tf/tf.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>




class VIOAttitudeLocalization
{
public:

    VIOAttitudeLocalization(ros::NodeHandle &);

private:

    ros::NodeHandle & nh_;

    ros::Publisher pubOdom_;

    // initial global body to nav attitude, from truth
    tf::Matrix3x3 R_imu_nav_o_;

    //relative attitude from quaternion in imu topic 
    tf::Matrix3x3 R_body_imu_;

    // pose estimate, saved over time to integrate position
    geometry_msgs::Pose pose_;

    bool firstKimera_;

    bool firstIMU_;

    ros::Time lastTime_;

    ros::Subscriber subKimera_;
    ros::Subscriber subImu_;

    double rollInc_;
    double pitchInc_;
    double yawInc_;
    double incCounter_;

    void kimeraCallback_(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback_(const sensor_msgs::Imu::ConstPtr& msg);


 };

#endif 
