#ifndef IMU_MAG_H
#define IMU_MAG_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "tf/tf.h"

#define IMU_MAG_DEFAULT_ACC_COVARIANCE          0.0000001
#define IMU_MAG_DEFAULT_GYR_COVARIANCE          0.000000029 //pow(0.00017,2) 0.00017 rad per sec = 0.01 degrees per sec
#define IMU_MAG_DEFAULT_MAG_COVARIANCE          0.00001
#define IMU_MAG_DEFAULT_IMU_POSE_COVARIANCE     -1          //sensor doesn't have orientation
#define IMU_MAG_DEFAULT_IMU_FRAME_ID            "odom"
#define IMU_MAG_GRAVITATIONAL_ACC               9.80665



class imu_mag {
  public:
    void update(int acc_x, int acc_y, int acc_z,
                int gyro_x, int gyro_y, int gyro_z,
                int mag_x, int mag_y, int mag_z);
    sensor_msgs::Imu imu_msg();
    sensor_msgs::MagneticField mag_msg();

    void tf_msg();

    void set_imu_frame_id(char*);
    void set_mag_frame_id (char*);
    void set_imu_acc_scale(double);
    void set_imu_gyro_scale(double);
    void set_mag_scale(double);

    imu_mag(double acc_multiplier, double acc_cov, 
            double gyro_multiplier, double gyro_cov,
            double mag_multiplier, double mag_cov);
  private:
    imu_mag();

    std::string  imu_frame_id, imu_child_frame, mag_frame_id;
    ros::Time current_time;
    double acc_scale, gyro_scale, mag_scale;
    double acc_covar, gyro_covar, mag_covar;
    int ax, ay, az, gx, gy, gz, mx, my, mz;

};

#endif

