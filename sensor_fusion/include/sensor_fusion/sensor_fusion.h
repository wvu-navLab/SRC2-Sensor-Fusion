#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H


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
#include <std_srvs/Trigger.h>
// Custom message includes. Auto-generated from msg/ directory.
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Int64.h>
#include <tf/transform_broadcaster.h>
//#include <kimera_vio_ros/KimeraVioRos.h>
#include <sensor_fusion/GetTruePose.h>
#include "srcp2_msgs/LocalizationSrv.h"
#define INITIALIZED 1
#define NOT_INITIALIZED 0



class SensorFusion
{
public:

    SensorFusion(ros::NodeHandle &);
    void initializationStatus_();

private:


    ros::NodeHandle & nh_;

    ros::Publisher pubOdom_, pubStatus_;

    ros::ServiceClient src2GetTruePoseClient_;
    ros::ServiceServer getTruePoseServer_;
    bool getTruePoseFromSRC2_(sensor_fusion::GetTruePose::Request &req, sensor_fusion::GetTruePose::Response &res);

    // initial global body to nav attitude, from truth
    tf::Matrix3x3 R_imu_nav_o_;

    //relative attitude from quaternion in imu topic
    tf::Matrix3x3 R_body_imu_;

    tf::Matrix3x3 Rbn_;

    tf::Vector3 v_body_;

    // pose estimate, saved over time to integrate position
    geometry_msgs::Pose pose_;

    bool firstWO_;
    bool firstVO_;
    bool firstIMU_;
    bool init_true_pose_;

    int initialized_;

    ros::Time lastTime_wo_;
    ros::Time lastTime_vo_;

    ros::Subscriber subVO_;
    ros::Subscriber subImu_;
    ros::Subscriber subWheelOdom_;
    ros::Subscriber subPositionUpdate_;

    double rollInc_;
    double pitchInc_;
    double yawInc_;
    double incCounter_;

    void voCallback_(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback_(const sensor_msgs::Imu::ConstPtr& msg);
    void wheelOdomCallback_(const nav_msgs::Odometry::ConstPtr& msg);
    void positionUpdateCallback_(const geometry_msgs::Pose::ConstPtr& msg);

    Eigen::Matrix <double, 6, 1> x_;
    Eigen::Matrix <double, 6, 6> P_;
    Eigen::Matrix <double, 6, 6> Q_;
    Eigen::Matrix <double, 3, 3> Rwo_;
    Eigen::Matrix <double, 3, 3> Rvo_;
    Eigen::Matrix <double, 3, 6> Hodom_;
    Eigen::Matrix <double, 6, 6> F_;
    Eigen::Matrix <double, 3,1> zVO_;
    Eigen::Matrix <double, 3,1> zWO_;

    Eigen::Matrix <double, 2, 6> Hposition_;
    Eigen::Matrix <double, 2, 1> zPosition_;
    Eigen::Matrix <double, 2, 2> Rposition_;

    void publishOdom_();

    tf::TransformBroadcaster odom_broadcaster_;

    bool averageIMU_;

    std::string odometry_frame_id;
    std::string odometry_child_frame_id;
    std::string position_update_topic;
    ros::ServiceClient clt_restart_kimera_;



 };

#endif
