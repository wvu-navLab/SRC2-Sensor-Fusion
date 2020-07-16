/*!
 * \kf_attitude_gyro.h
 * \brief kf_attitude_gyro (...).
 *
 * Kalman filter gyro for attitude prediction, low noise hf, and attitude measurements update, noisy  (...).
 *
 * \author Bernardo Martinez Rocamora Junior, WVU - bm00002@mix.wvu.edu
 * \date July 14, 2020
 */

#include "vio_attitude_localization/kf_attitude_gyro.h"

KFAttitudeGyro::KFAttitudeGyro(ros::NodeHandle & nh)
	: nh_(nh)
{
	dt_ = 0.2;
    ros::Time time_now = ros::Time::now();

    double sigma_rp = 0.2;
    double sigma_y = 1.5;
    double sigma_biases = 0.1;
        
    x_  <<  0, 
            0, 
            0, 
            0, 
            0, 
            0;

    P_  <<  pow(sigma_rp,2), 0, 0, 0, 0, 0, 
            0, pow(sigma_rp,2), 0, 0, 0, 0,
            0, 0, pow(sigma_y,2), 0, 0, 0,
            0, 0, 0, pow(sigma_biases,2), 0, 0,
            0, 0, 0, 0, pow(sigma_biases,2), 0,
            0, 0, 0, 0, 0, pow(sigma_biases,2);
         
	subImu = nh_.subscribe("imu",10,&KFAttitudeGyro::imuCallback,this);
    subOdom = nh_.subscribe("odometry/truth", 1000, &KFAttitudeGyro::odometryCallback, this);

    pubOdomRoll = nh_.advertise<geometry_msgs::PointStamped>("attitude/odom/roll",1);
    pubOdomPitch = nh_.advertise<geometry_msgs::PointStamped>("attitude/odom/pitch",1);
    pubOdomYaw = nh_.advertise<geometry_msgs::PointStamped>("attitude/odom/yaw",1);

    pubRollMeasured = nh_.advertise<geometry_msgs::PointStamped>("attitude/measured/roll",1);
    pubPitchMeasured = nh_.advertise<geometry_msgs::PointStamped>("attitude/measured/pitch",1);
    pubYawMeasured = nh_.advertise<geometry_msgs::PointStamped>("attitude/measured/yaw",1);

    pubRollEstimated = nh_.advertise<geometry_msgs::PointStamped>("attitude/estimated/roll",1);
    pubPitchEstimated = nh_.advertise<geometry_msgs::PointStamped>("attitude/estimated/pitch",1);
    pubYawEstimated = nh_.advertise<geometry_msgs::PointStamped>("attitude/estimated/yaw",1);

    pubBiasesEstimated = nh_.advertise<geometry_msgs::PointStamped>("attitude/estimated/gyro_biases",1);
    
    pubImuFiltered = nh_.advertise<sensor_msgs::Imu>("imu_filtered",1);

    last_time_ = time_now.toSec();

}

void KFAttitudeGyro::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    ros::Time time_now = msg->header.stamp;

	double roll_odom, pitch_odom, yaw_odom;
    tf::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w
                    );

    tf::Matrix3x3 m(q);
    m.getRPY(roll_odom, pitch_odom, yaw_odom);

    publishStates(roll_odom, 0, time_now,  pubOdomRoll);
    publishStates(pitch_odom, 0, time_now,  pubOdomPitch);
    publishStates(yaw_odom, 0, time_now,  pubOdomYaw);
}


void KFAttitudeGyro::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion q(
                    msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w
                    );

    tf::Matrix3x3 m(q);

    ros::Time time_now = msg->header.stamp;

    dt_ = time_now.toSec() - last_time_;

    // ROS_INFO_STREAM ("Delta t: " << dt_ );

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double p_til, q_til, r_til;
    p_til = msg->angular_velocity.x - x_[3];
    q_til = msg->angular_velocity.y - x_[4];
    r_til = msg->angular_velocity.z - x_[5];

    Eigen::Matrix <double, 3, 1> omega;
	omega 	<< 	p_til,
             	q_til,
             	r_til;

    Eigen::Matrix <double, 3, 3> Cib;
    Cib	<< 	1, sin(x_(0))*tan(x_(1)),   cos(x_(0))*tan(x_(1)),
			0, cos(x_(0)),              -sin(x_(0)),
			0, sin(x_(0))/cos(x_(1)),   cos(x_(0))/cos(x_(1));

    Eigen::Matrix <double, 3, 1> angular_rates;
	angular_rates = Cib*omega; 


    if (std::signbit(yaw*x_(2)))
    {
        if (yaw > M_PI/2)
        {
            yaw = yaw - 2 * M_PI;
        }
        else if (yaw < -M_PI/2)
        {
            yaw = yaw + 2 * M_PI;
        }
    }
 
    // Non-linear update of state vector
    x_(0) = x_(0) + angular_rates(0) * dt_;
    x_(1) = x_(1) + angular_rates(1) * dt_;
    x_(2) = x_(2) + angular_rates(2) * dt_;

    // Jacobian wrt to state vector
    Eigen::Matrix <double, 6, 6> F = Eigen::MatrixXd::Zero(6,6);
    F(0,0) = 1 + (cos(x_(0))*tan(x_(1))*q_til - sin(x_(0))*tan(x_(1))*r_til) * dt_;
    F(0,1) = (sin(x_(0))/cos(x_(1))/cos(x_(1))*q_til + cos(x_(0))/cos(x_(1))/cos(x_(1))*r_til) * dt_;
    F(0,3) = - dt_;
    F(0,4) = (- sin(x_(0))*tan(x_(0))) * dt_;
    F(0,5) = (- cos(x_(0))*tan(x_(0))) * dt_;

    F(1,0) = (sin(x_(0))*q_til + cos(x_(0))*r_til) * dt_;
	F(1,1) = 1;
    F(1,4) = (- cos(x_(0))) * dt_;
    F(1,5) = (sin(x_(0))) * dt_;

    F(2,0) = (cos(x_(0))/cos(x_(1))*q_til - sin(x_(0))/cos(x_(1))*r_til) * dt_;
    F(2,1) = (sin(x_(0))*tan(x_(1))/cos(x_(1))*q_til + cos(x_(0))*tan(x_(1))/cos(x_(1))*r_til) * dt_;
    F(2,2) = 1;
    F(2,3) = (- sin(x_(0))/cos(x_(1))) * dt_;
    F(2,5) = (- cos(x_(0))/cos(x_(1))) * dt_;

    F(3,3) = 1;
    F(4,4) = 1;
    F(5,5) = 1;

    // Jacobian wrt to input vector
    Eigen::Matrix <double, 6, 3> G = Eigen::MatrixXd::Zero(6,3);
    G(0,0) = dt_;
    G(0,1) = (sin(x_(0))*tan(x_(0))) * dt_;
    G(0,2) = (cos(x_(0))*tan(x_(0))) * dt_;

    G(1,0) = 0;
    G(1,1) = (cos(x_(0))) * dt_;
    G(1,2) = (- sin(x_(0))) * dt_;   

    G(2,0) = 0;
    G(2,1) = (sin(x_(0))/cos(x_(1))) * dt_;
    G(2,2) = (cos(x_(0))/cos(x_(1))) * dt_;

	double sigma_attitude = 0.05;
    Eigen::Matrix <double, 3, 3> Q;
    Q   <<  pow(sigma_attitude,2), 0, 0,
            0, pow(sigma_attitude,2), 0,
            0, 0, pow(sigma_attitude,2);

    // Covariance Prediction
    P_ = F * P_ * F.transpose() + G * Q * G.transpose();

    Eigen::Matrix <double, 3, 6> H;
    H   <<  1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0;

    Eigen::Matrix <double, 3, 1> z;
   	z   <<  roll, pitch, yaw;

    if (fabs(x_(2)) > M_PI)
    {
        std::cout << "Roll Over Detected" << std::endl;
        if(x_(2) > 0) 
        {
            x_(2) = x_(2) - 2*M_PI;
        }
        else 
        {
            x_(2) = x_(2) + 2*M_PI;
        }
    }

    // Innovation
    Eigen::Matrix <double, 3, 1> y;
    y = z - H * x_;

    double sigma_att_direct = 0.1;
	Eigen::Matrix <double, 3, 3> R;
    R   <<  pow(sigma_att_direct,2), 0, 0,
            0, pow(sigma_att_direct,2), 0,
            0, 0, pow(sigma_att_direct,2);

    // Innovation Covariance
    Eigen::Matrix <double, 3, 3> S;
    S = H * P_ * H.transpose() + R;

    // Kalman Gain
    Eigen::Matrix <double, 6, 3> K;
    K = P_ * H.transpose() * S.inverse();

    // State Update
    x_ = x_ + K * y;

    // Yaw wrapper
    if (fabs(x_(2)) > M_PI)
    {
        if(x_(2) > 0) 
        {
            x_(2) = x_(2) - 2*M_PI;
        }
        else 
        {
            x_(2) = x_(2) + 2*M_PI;
        }
    }

    // Covariance Update
    P_ = (Eigen::MatrixXd::Identity(6,6) - K * H) * P_;

    publishStates(roll, 0, time_now,  pubRollMeasured);
    publishStates(pitch, 0, time_now,  pubPitchMeasured);
    publishStates(yaw, 0, time_now,  pubYawMeasured);

    publishStates(x_(0), P_(0,0), time_now,  pubRollEstimated);
    publishStates(x_(1), P_(1,1), time_now,  pubPitchEstimated);
    publishStates(x_(2), P_(2,2), time_now,  pubYawEstimated);
    publishBiases(x_(3), x_(4), x_(5), time_now,  pubBiasesEstimated);

    last_time_ = time_now.toSec();

    sensor_msgs::Imu new_msg = *msg;

    tf::Quaternion new_q;
    new_q.setRPY(x_(0),x_(1),x_(2));
    new_q.normalize();

    new_msg.orientation.x = new_q.x();
    new_msg.orientation.y = new_q.y();
    new_msg.orientation.z = new_q.z();
    new_msg.orientation.w = new_q.w(); 

    new_msg.orientation_covariance = {  P_(0,0), P_(0,1), P_(0,2),
                                        P_(1,0), P_(1,1), P_(1,2),
                                        P_(2,0), P_(2,1), P_(3,2),};
    pubImuFiltered.publish(new_msg);
}

// Publish estimated states in global frame
void KFAttitudeGyro::publishStates(const double & state, const double & covariance, const ros::Time & time, const ros::Publisher & pub)
{
    geometry_msgs::PointStamped msg;

    msg.header.stamp = time;
    msg.header.frame_id = "/scout_1_tf/odom";
    msg.point.x = state;
    msg.point.y = state + 3*sqrt(covariance);
    msg.point.z = state - 3*sqrt(covariance);

    pub.publish(msg);
}

// Publish estimated states in global frame
void KFAttitudeGyro::publishBiases(const double & bx, const double & by, const double & bz, const ros::Time & time, const ros::Publisher & pub)
{
    geometry_msgs::PointStamped msg;

    msg.header.stamp = time;
    msg.header.frame_id = "/scout_1_tf/odom";
    msg.point.x = bx;
    msg.point.y = by;
    msg.point.z = bz;

    pub.publish(msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "kf_attitude_gyro");
	ros::NodeHandle nh("");

	ROS_INFO("IMU RPY Noise measurements started ");

	KFAttitudeGyro kf_attitude_gyro(nh);

	ros::spin();

	return 0;
}