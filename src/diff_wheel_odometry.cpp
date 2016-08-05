#include "diff_pi_bot/diff_wheel_odometry.h"

// wheel separation (m), x location, y location, linear velocity, angular velocity, encoder reading left, encoder reading right
diff_wheel_odometry::diff_wheel_odometry(double wheel_separation, double counts_per_meter, 
    double x0, double y0, double theta0, double v0, double w0, double left0, double right0,
    double pose_cov, double twist_cov):
    separation(wheel_separation), counts_per_m(counts_per_meter), x(x0), y(y0),
    theta(theta0), v(v0), w(w0), l(left0), r(right0), pose_covar(pose_cov),
    twist_covar(twist_cov) {
  s_last_time = ros::Time::now();
  s_last_l = l;
  s_last_r = r;
  odom_frame_id = "odom";
  odom_child_frame_id = "base_footprint";
  turn_multiplier = 1;
}

void diff_wheel_odometry::update(double l, double r) {

  l/=counts_per_m;
  r/=counts_per_m;
  current_time = ros::Time::now();
  
  // Time difference
  double dt = (current_time - s_last_time).toSec();
  s_last_time = current_time;
  
  // Compute distance change from each wheel
  double dl = l - s_last_l;
  s_last_l = l;
  double dr = r - s_last_r;
  s_last_r = r;
  
  double dxy = (dl + dr) / 2;
  double dtheta = (((dr - dl) / separation) * turn_multiplier);
  
  // Cartesian coordinates
  x += dxy * cosf(theta);
  y += dxy * sinf(theta);
  
  theta += dtheta;
  
  // Compute velocity
  v = dxy / dt;
  w = dtheta  / dt;
}

nav_msgs::Odometry diff_wheel_odometry::odom_msg () {
  nav_msgs::Odometry ret;
  ret.header.stamp = current_time;
  ret.header.frame_id = odom_frame_id;

  // Set the position
  ret.pose.pose.position.x = x;
  ret.pose.pose.position.y = y;
  ret.pose.pose.position.z = 0.0;
  ret.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);
  ret.pose.covariance = { pose_covar, 0, 0, 0, 0, 0,
                          0, pose_covar, 0, 0, 0, 0,
                          0, 0, pose_covar, 0, 0, 0,
                          0, 0, 0, pose_covar, 0, 0,
                          0, 0, 0, 0, pose_covar, 0,
                          0, 0, 0, 0, 0, pose_covar};

  // Set the velocity
  ret.child_frame_id = odom_child_frame_id;
  ret.twist.twist.linear.x = v;
  ret.twist.twist.linear.y = 0.0;
  ret.twist.twist.linear.z = 0.0;
  ret.twist.twist.angular.x = 0.0;
  ret.twist.twist.angular.y = 0.0;
  ret.twist.twist.angular.z = w;
  ret.twist.covariance = {twist_covar, 0, 0, 0, 0, 0,
                          0, twist_covar, 0, 0, 0, 0,
                          0, 0, twist_covar, 0, 0, 0,
                          0, 0, 0, twist_covar, 0, 0,
                          0, 0, 0, 0, twist_covar, 0,
                          0, 0, 0, 0, 0, twist_covar};

  return ret;
}

geometry_msgs::TransformStamped diff_wheel_odometry::tf_msg() {
    geometry_msgs::TransformStamped ret;

    ret.header.stamp = current_time;
    ret.header.frame_id = odom_frame_id;
    ret.child_frame_id = odom_child_frame_id;

    ret.transform.translation.x = x;
    ret.transform.translation.y = y;
    ret.transform.translation.z = 0.0;
    ret.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
  return ret;
}

// set odometry frame ID, default: odom
void diff_wheel_odometry::set_odom_frame_id(const string frame_id) {
  odom_frame_id.assign(frame_id);
}

// set odometry child frame ID, default: base_link
void diff_wheel_odometry::set_odom_child_frame_id (const string frame_id) {
  odom_child_frame_id.assign(frame_id);
}

void diff_wheel_odometry::set_wheel_separation(double wheel_separation) {
  separation = wheel_separation;
}

void diff_wheel_odometry::set_turn_multiplier(double multilier) {
  turn_multiplier = multilier;
}