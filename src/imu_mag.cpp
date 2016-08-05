#include "diff_pi_bot/imu_mag.h"


imu_mag::imu_mag() {

}

imu_mag::imu_mag(double acc_multiplier, double acc_cov, 
                double gyro_multiplier, double gyro_cov,
                double mag_multiplier, double mag_cov)
                : acc_scale(acc_multiplier), acc_covar(acc_cov),
                gyro_scale(gyro_multiplier), gyro_covar(gyro_cov),
                mag_scale(mag_multiplier), mag_covar(mag_cov) {
	imu_frame_id = IMU_MAG_DEFAULT_IMU_FRAME_ID;

}


void imu_mag::update(int acc_x, int acc_y, int acc_z, int gyro_x, int gyro_y, int gyro_z, int mag_x, int mag_y, int mag_z) {
    current_time = ros::Time::now();
    ax = acc_x;
    ay = acc_y;
    az = acc_z;
    gx = gyro_x;
    gy = gyro_y;
    gz = gyro_z;
    mx = mag_x;
    my = mag_y;
    mz = mag_z;
}

sensor_msgs::Imu imu_mag::imu_msg() {
    sensor_msgs::Imu ret;
    ret.header.stamp = current_time;
    ret.header.frame_id = imu_frame_id;
    ret.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    ret.orientation_covariance = {IMU_MAG_DEFAULT_IMU_POSE_COVARIANCE, 0, 0,  0, IMU_MAG_DEFAULT_IMU_POSE_COVARIANCE, 0,  0, 0, IMU_MAG_DEFAULT_IMU_POSE_COVARIANCE}; 
    ret.angular_velocity.x = gx * gyro_scale;
    ret.angular_velocity.y = gy * gyro_scale;
    ret.angular_velocity.z = gz * gyro_scale;
    ret.angular_velocity_covariance = {gyro_covar, 0, 0,  0, gyro_covar, 0,  0, 0, gyro_covar};
    ret.linear_acceleration.x = ax * acc_scale;
    ret.linear_acceleration.y = ay * acc_scale;
    ret.linear_acceleration.z = az * acc_scale;
    ret.linear_acceleration_covariance = {acc_covar, 0, 0,  0, acc_covar, 0,  0, 0, acc_covar};
    return ret;
}

sensor_msgs::MagneticField imu_mag::mag_msg(){
	sensor_msgs::MagneticField ret;
    ret.header.stamp = current_time;
    ret.header.frame_id = imu_frame_id;
    ret.magnetic_field.x = mx;
    ret.magnetic_field.x = my;
    ret.magnetic_field.x = mz;
    ret.magnetic_field_covariance = {mag_covar, 0, 0,  0, mag_covar, 0,  0, 0, mag_covar};
    return ret;
}

// set imu frame ID, default: odom
void imu_mag::set_imu_acc_scale(double scale) {
    acc_scale = scale;
}

// set imu frame ID, default: odom
void imu_mag::set_imu_gyro_scale(double scale) {
    gyro_scale = scale;
}

// set mag frame ID, default: TODO
void imu_mag::set_mag_frame_id(char* frame_id) {
    mag_frame_id.assign(frame_id);
}

void imu_mag::set_mag_scale(double scale) {
    mag_scale = scale;
}