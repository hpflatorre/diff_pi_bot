#ifndef DIFF_WHEEL_ODOMETRY_H
#define DIFF_WHEEL_ODOMETRY_H

#include <cstdio>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

using std::string;

#define DIFF_WHEEL_ODOMETRY_DEFAULT_POSE_COVARIANCE         0
#define DIFF_WHEEL_ODOMETRY_DEFAULT_TWIST_COVARIANCE        0

class diff_wheel_odometry {
  public:
	void update(double left, double right);
	nav_msgs::Odometry odom_msg();
	geometry_msgs::TransformStamped tf_msg();
	//void set_odom_topic(char*);
	void set_odom_frame_id(const string);
	void set_odom_child_frame_id (const string);
	void set_wheel_separation(double);
	void set_turn_multiplier(double);
	//void initial_reading();

	diff_wheel_odometry(double wheel_separation, double counts_per_meter, 
		double x0, double y0, double theta0, double v0, double w0, double left0,
		double right0, double pose_cov, double twist_cov);
  private:
	double l, r, separation, turn_multiplier, counts_per_m;
	// x location, y location, linear velocity, wheel separation (m), angular velocity (z axis), yaw, last encored reading left, last encored reading right
	double x, y, v, w, theta, s_last_l, s_last_r, pose_covar, twist_covar;
	std::string odom_frame_id, odom_child_frame_id;
	ros::Time current_time, s_last_time;
	
	diff_wheel_odometry();
	
};

#endif
