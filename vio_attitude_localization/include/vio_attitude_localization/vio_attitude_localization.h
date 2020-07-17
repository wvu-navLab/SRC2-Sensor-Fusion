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
#include <eigen3/Eigen/Dense>




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

    tf::Matrix3x3 Rbn_;

    // pose estimate, saved over time to integrate position
    geometry_msgs::Pose pose_;

    bool firstKimera_;

    bool firstIMU_;

    ros::Time lastTime_;

    ros::Subscriber subKimera_;
    ros::Subscriber subImu_;
    ros::Subscriber subWheelOdom_;

    double rollInc_;
    double pitchInc_;
    double yawInc_;
    double incCounter_;

    void kimeraCallback_(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback_(const sensor_msgs::Imu::ConstPtr& msg);
    void wheelOdomCallback_(const nav_msgs::Odometry::ConstPtr& msg);


    Eigen::Matrix <double, 6, 1> x_;
    Eigen::Matrix <double, 6, 6> P_;
    Eigen::Matrix <double, 6, 6> Q_;
    Eigen::Matrix <double, 2, 2> Rwo_;
    Eigen::Matrix <double, 3, 3> Rvio_;
    Eigen::Matrix <double, 3, 6> Hvio_;
    Eigen::Matrix <double, 6, 6> F_;
    Eigen::Matrix <double, 3,1> zVIO_;
    Eigen::Matrix <double, 2,1> zWO_;
    Eigen::Matrix <double, 2,2> Rbn2x2_;


 };

#endif 
