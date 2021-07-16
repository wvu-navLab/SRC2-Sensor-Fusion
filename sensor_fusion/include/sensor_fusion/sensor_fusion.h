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
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Int64.h>
#include <tf/transform_broadcaster.h>
//#include <kimera_vio_ros/KimeraVioRos.h>
#include <sensor_fusion/GetTruePose.h>
#include "srcp2_msgs/LocalizationSrv.h"
#include <std_srvs/Trigger.h>
#define INITIALIZED 1
#define NOT_INITIALIZED 0
#define MOBILE 1
#define IMMOBILE 0



class SensorFusion
{
public:

    SensorFusion(ros::NodeHandle &);
    void initializationStatus_();
    void PublishInitAttitude();
    bool have_init_attitude =false;
    bool publish_attitude= false;

private:
   bool averageAccel_;
   double accelCount_;
    ros::NodeHandle & nh_;

    ros::Publisher pubOdom_, pubStatus_, pubSlip_, pubMobility_, pubInitAttitude_;


    ros::ServiceClient src2GetTruePoseClient_;

    ros::ServiceServer getTruePoseServer_;
    bool getTruePoseFromSRC2_(sensor_fusion::GetTruePose::Request &req, sensor_fusion::GetTruePose::Response &res);
    geometry_msgs::Quaternion q_msg;
    // initial global body to nav attitude, from truth
    tf::Matrix3x3 R_imu_nav_o_;

    //relative attitude from quaternion in imu topic
    tf::Matrix3x3 R_body_imu_;

    tf::Matrix3x3 Rbn_;
    tf::Vector3 vb_wo_, vb_vo_;
    Eigen::Matrix <double, 3, 1> vAccNav_;
    Eigen::Matrix <double, 3, 1> accelIMU_;
    Eigen::Matrix <double, 3, 1>  g_;
    // pose estimate, saved over time to integrate position
    geometry_msgs::Pose pose_;

    bool firstWO_;
    bool firstVO_;
    bool firstIMU_;
    bool init_true_pose_;
    bool init_true_position_;
    bool init_true_attitude_;
    bool high_slip_flag_= false;
    bool homingUpdateFlag_=false;
    bool true_pose_from_src2 =false;
    int initialized_;
    int slipCount_=0;
    std_msgs::Int64 status_;
    std_msgs::Int64 mobility_;

    ros::Time lastTime_wo_;
    ros::Time lastTime_vo_;
    ros::Time slipTimer_;
    ros::Time start_time_dist;

    ros::Subscriber subVO_;
    ros::Subscriber subImu_;
    ros::Subscriber subWheelOdom_;
    ros::Subscriber subPositionUpdate_;
    ros::Subscriber subDrivingMode_;
    ros::Subscriber subInitAttitude_;

    double init_pos_z_;
    double init_x;
    double init_y;
    double rollInc_;
    double pitchInc_;
    double yawInc_;
    double incCounter_;
    double slipTimer =0;
    double yawTimer=0;
    double yaw_pre=0;
    double distDiff=0;
    double distNow=0;
    double distPrev=0;
    double temp_x_;
    double temp_y_;
    double slipped_x_;
    double slipped_y_;
    double x_diff_;
    double y_diff_;
    void voCallback_(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback_(const sensor_msgs::Imu::ConstPtr& msg);
    void wheelOdomCallback_(const nav_msgs::Odometry::ConstPtr& msg);
    void positionUpdateCallback_(const geometry_msgs::Pose::ConstPtr& msg);
    void drivingModeCallback_(const std_msgs::Int64::ConstPtr& msg);
    void attitudeInitCallback_(const geometry_msgs::Quaternion::ConstPtr& msg);

    Eigen::Matrix <double, 6, 1> x_;
    Eigen::Matrix <double, 6, 1> last_x_;
    Eigen::Matrix <double, 6, 6> P_;
    Eigen::Matrix <double, 6, 6> Q_;
    Eigen::Matrix <double, 3, 3> Rwo_;
    Eigen::Matrix <double, 3, 3> Rvo_;
    Eigen::Matrix <double, 3, 6> Hodom_;
    Eigen::Matrix <double, 6, 6> F_;
    Eigen::Matrix <double, 3,1> zVO_;
    Eigen::Matrix <double, 3,1> zWO_;

    Eigen::Matrix <double, 6, 6> Hposition_;
    Eigen::Matrix <double, 6, 1> zPosition_;
    Eigen::Matrix <double, 6, 6> Rposition_;

    void publishOdom_();
    int driving_mode_;
    double slip_;

    tf::TransformBroadcaster odom_broadcaster_;

    bool averageIMU_;

    std::string odometry_frame_id;
    std::string odometry_child_frame_id;
    std::string position_update_topic;
    std::string robot_name;

    ros::ServiceClient clt_restart_kimera_;



 };

#endif
