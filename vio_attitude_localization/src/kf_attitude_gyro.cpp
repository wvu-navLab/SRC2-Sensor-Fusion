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

    double sigma_bank = 0.1;
    double sigma_heading = 1.5;
    double sigma_biases = 1.5;
    
    x_  <<  0, 0, 0, 0, 0, 0;
    P_  <<  pow(sigma_bank,2), 0, 0, 0, 0, 0, 
            0, pow(sigma_bank,2), 0, 0, 0, 0,
            0, 0, pow(sigma_heading,2), 0, 0, 0,
            0, 0, 0, pow(sigma_biases,2), 0, 0,
            0, 0, 0, 0, pow(sigma_biases,2), 0,
            0, 0, 0, 0, 0, pow(sigma_biases,2);
            
	subImu = nh_.subscribe("/scout_1/imu",10,&KFAttitudeGyro::imuCallback,this);

    pubRoll = nh_.advertise<std_msgs::Float64>("/scout_1/imu_processed/roll",1);
    pubPitch = nh_.advertise<std_msgs::Float64>("/scout_1/imu_processed/pitch",1);
    pubYaw = nh_.advertise<std_msgs::Float64>("/scout_1/imu_processed/yaw",1);
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

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Eigen::Matrix <double, 3, 1> y;
    y   <<  roll,
            pitch,
            yaw;

    double p_til, q_til, r_til;
    p_til = msg->angular_velocity.x - x_[3];
    q_til = msg->angular_velocity.y - x_[4];
    r_til = msg->angular_velocity.z - x_[5];

    Eigen::Matrix <double, 3, 1> omega;
	omega 	<< 	p_til,
             	q_til,
             	r_til;

    Eigen::Matrix <double, 3, 3> Cib;
    Cib	<< 	sin(x_(0))*sin(x_(1))/cos(x_(1)), 	1, 	-cos(x_(0))*sin(x_(1))/cos(x_(1)),
			cos(x_(0)),							0,	sin(x_(0)),
			sin(x_(0))/cos(x_(1)),				0,	-cos(x_(0))/cos(x_(1));

    Eigen::Matrix <double, 3, 1> angular_rates;
	angular_rates = Cib*omega; 
   
    Eigen::Matrix <double, 6, 6> F = Eigen::MatrixXd::Zero(6,6);
    F(0,0) = 1 + (cos(x_(0))*tan(x_(1))*p_til + sin(x_(0))*tan(x_(1))*r_til) * dt_;
    F(0,1) = (sin(x_(0))/cos(x_(1))/cos(x_(1))*p_til - cos(x_(0))/cos(x_(1))/cos(x_(1))*r_til) * dt_;
    F(0,3) = (-sin(x_(0))*tan(x_(1))) * dt_;
    F(0,4) = dt_;
    F(0,5) = (cos(x_(0))*tan(x_(1))) * dt_;

    F(1,0) = (-sin(x_(0))*p_til + cos(x_(0))*r_til) * dt_;
	F(1,1) = 1;
    F(1,3) = (-cos(x_(0))) * dt_;
    F(1,5) = (-sin(x_(0))) * dt_;

    F(2,0) = (cos(x_(0))/cos(x_(1))*p_til + sin(x_(0))/cos(x_(1))*r_til) * dt_;
    F(2,1) = (sin(x_(0))*tan(x_(1))/cos(x_(1))*p_til - cos(x_(0))*tan(x_(1))/cos(x_(1))*r_til) * dt_;
    F(2,2) = 1;
    F(2,3) = (sin(x_(0))/cos(x_(1))) * dt_;
    F(2,5) = (sin(x_(0))/cos(x_(1))) * dt_;

    F(3,3) = 1;
    F(4,4) = 1;
    F(5,5) = 1;

    // State Prediction
    x_(0) = x_(0) + angular_rates(0) * dt_;
    x_(1) = x_(1) + angular_rates(1) * dt_;
    x_(2) = x_(2) + angular_rates(2) * dt_;

	double sigma_attitude = 0.05;
    Eigen::Matrix <double, 6, 6> Q;
    Q   <<  pow(sigma_attitude,2), 0, 0, 0, 0, 0,
            0, pow(sigma_attitude,2), 0, 0, 0, 0,
            0, 0, pow(sigma_attitude,2), 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;

    // Covariance Prediction
    P_ = F * P_ * F.transpose() + Q;

    Eigen::Matrix <double, 3, 6> H;
    H   <<  1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0;

    Eigen::Matrix <double, 3, 1> z;
   	z   <<  roll, pitch, yaw;

    // Innovation
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

    // Covariance Update
    P_ = (Eigen::MatrixXd::Identity(6,6) - K * H) * P_;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kf_attitude_gyro");
	ros::NodeHandle nh("");

	ROS_INFO(" IMU RPY Noise measurements started ");

	KFAttitudeGyro kf_attitude_gyro(nh);

	ros::spin();

	return 0;
}