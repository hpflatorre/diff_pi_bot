#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include <sstream>
#include <cstdio>
#include <serial/serial.h>

#include "shelf_bot/diff_wheel_odometry.h"
#include "shelf_bot/imu_mag.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

// Serial protocol flags
#define MSG_END_BYTE        '\n'
#define MSG_FIELD_SEPARATOR ';'
#define MSG_ODOMETRY        'o'
#define MSG_IMU             'i'

// DEFAULTS
#define DEFAULT_ODOM_POSE_COVARIANCE          0.00001
#define DEFAULT_ODOM_TWIST_COVARIANCE         0.00001
#define DEFAULT_SERIAL_PORT                   "/dev/ttyUSB0"
#define DEFAULT_SERIAL_BAUD                   115200
#define DEFAULT_ENCODER_PULSE_METER_RATIO     181                               // 40 counts = 0.121m
#define DEFAULT_WHEEL_DISTANCE_M              0.13
#define DEFAULT_GAIN_ACC                      0.000061 * 9.80665                // FS = 2
#define DEFAULT_GAIN_GYRO                     0.00875 * 3.14159265359 / 180     // FS = 245
#define DEFAULT_GAIN_MAG                      1
#define DEFAULT_ACC_COVARIANCE                -1
#define DEFAULT_GYRO_COVARIANCE               -1
#define DEFAULT_MAG_COVARIANCE                -1
#define DEFAULT_ODOM_FRAME_ID                 "odom"
#define DEFAULT_ODOM_CHILD_FRAME_ID           "base_footprint"
#define DEFAULT_TOPIC_CMD_VEL                 "cmd_vel"
#define DEFAULT_TOPIC_ODOM                    "odom"
#define DEFAULT_TOPIC_IMU                     "imu/data_raw"
#define DEFAULT_TOPIC_MAG                     "imu/mag"

// Global Variables:
volatile double g_vel_l(0), g_vel_r(0), g_wheel_distance(DEFAULT_WHEEL_DISTANCE_M), g_turn_multiplier(); 

void print_usage() {
  cerr << "Usage: shelf_bot shelf_bot_node {-e|<serial port address>} ";
  cerr << "<baudrate> " << endl;
  cerr << "<node_id>" << endl;
}

void enumerate_ports() {
  vector<serial::PortInfo> devices_found = serial::list_ports();
  vector<serial::PortInfo>::iterator iter = devices_found.begin();
  while( iter != devices_found.end() ) {
    serial::PortInfo device = *iter++;
    cout << device.port.c_str() << ", " << device.description.c_str() << ", " << device.hardware_id.c_str() << endl;
  }
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  //ROS_INFO("I heard: [vlx: %f; vaz: %f ]", msg->linear.x, msg->angular.z);
  g_vel_l = (msg->linear.x - g_wheel_distance * msg->angular.z / 2);
  g_vel_r = (msg->linear.x + g_wheel_distance * msg->angular.z / 2);
  ROS_INFO("Velocity: [vl: %f; vr: %f ]", g_vel_l, g_vel_r);
}


int main(int argc, char **argv) {
 
  // Check Arguments  //////////////////////////////////////////////////////////
  /*
  if(argc < 2) {
    print_usage();
    return 0;
  }

  // Argument 1 is the serial port or enumerate flag
  string port(argv[1]);
  
  if( port == "-e" ) {
    enumerate_ports();
      return 0;
  }
  else if( argc < 4 ) {
    print_usage();
    return 1;
  }
  */
  // Argument 2 is the baudrate
  int baud = 0;
  string port, topic_cmd_vel, topic_odom, topic_imu, topic_mag;
  string odom_frame_id, odom_child_frame_id;
  //sscanf(argv[2], "%lu", &baud);
  
  // Message handle variables
  std::string inputs, remainder, message, outputs;
  size_t index;
  double d1, d2, d3, d4, d5, d6, d7, d8, d9;

  // Parameters
  double odom_pose_cov, odom_twist_cov;
  double pulses_per_m, wheel_distance;
  double cov_acc, cov_gyro, cov_mag;
  double gain_acc, gain_gyro, gain_mag;

  // ROS ///////////////////////////////////////////////////////////////////////
  
  ros::init(argc, argv, "Diff_pi_bot_serial_interface");
  ros::NodeHandle n;
  
  // Get odometry model parameters
    ROS_INFO("Reading robot Parameters");

  if (!n.param<double>("odom/pose_covariance", odom_pose_cov, DEFAULT_ODOM_POSE_COVARIANCE))
    ROS_INFO("Using default odom/pose_covariance: %f", odom_pose_cov);

  if (!n.param<double>("odom/twist_covariance", odom_twist_cov, DEFAULT_ODOM_TWIST_COVARIANCE))
    ROS_INFO("Using default odom/twist_covariance: %f", odom_twist_cov);

  if (!n.param<double>("odom/encoder_puse_meter_ratio", pulses_per_m, DEFAULT_ENCODER_PULSE_METER_RATIO))
    ROS_INFO("Using default odom/encoder_puse_meter_ratio: %f", pulses_per_m);

  if (!n.param<double>("odom/wheel_distance", wheel_distance, DEFAULT_WHEEL_DISTANCE_M))
    ROS_INFO("Using default odom/wheel_distance: %fm", wheel_distance);

  if (!n.param<std::string>("odom/frame_id", odom_frame_id, DEFAULT_ODOM_FRAME_ID))
    ROS_INFO("Using default odom/child_frame_id: %s", odom_frame_id.c_str());

  if (!n.param<std::string>("odom/child_frame_id", odom_child_frame_id, DEFAULT_ODOM_CHILD_FRAME_ID))
    ROS_INFO("Using default odom/child_frame_id: %s", odom_child_frame_id.c_str());

  // Get IMU and Magnetometer parameters
  if (!n.param<double>("imu_mag/accelerometer_covariance", cov_acc, DEFAULT_ACC_COVARIANCE))
    ROS_INFO("Using default imu_mag/accelerometer_covariance: %f", cov_acc);

  if (!n.param<double>("imu_mag/gyro_covariance", cov_gyro, DEFAULT_GYRO_COVARIANCE))
    ROS_INFO("Using default imu_mag/gyro_covariance: %f", cov_gyro);

  if (!n.param<double>("imu_mag/magnetometer_covariance", cov_mag, DEFAULT_MAG_COVARIANCE))
    ROS_INFO("Using default imu_mag/magnetometer_covariance: %f", cov_mag);

  if (!n.param<double>("imu_mag/magnetometer_gain", gain_mag, DEFAULT_GAIN_ACC))
    ROS_INFO("Using default imu_mag/magnetometer_gain: %f", gain_mag);

  if (!n.param<double>("imu_mag/gyro_gain", gain_gyro, DEFAULT_GAIN_GYRO))
    ROS_INFO("Using default imu_mag/gyro_gain: %f", gain_gyro);

  if (!n.param<double>("imu_mag/magnetometer_gain", gain_mag, DEFAULT_GAIN_MAG))
    ROS_INFO("Using default imu_mag/magnetometer_gain: %f", gain_mag);

  // Topic names parameters
  if (!n.param<std::string>("cmd/topic_cmd_vel", topic_cmd_vel, DEFAULT_TOPIC_CMD_VEL))
    ROS_INFO("Using default cmd/topic_cmd_vel: %s", topic_cmd_vel.c_str());

  if (!n.param<std::string>("odom/topic_odom", topic_odom, DEFAULT_TOPIC_ODOM))
    ROS_INFO("Using default odom/topic_odom: %s", topic_odom.c_str());

  if (!n.param<std::string>("imu_mag/topic_imu", topic_imu, DEFAULT_TOPIC_IMU))
    ROS_INFO("Using default imu_mag/topic_imu: %s", topic_imu.c_str());

  if (!n.param<std::string>("imu_mag/topic_mag", topic_mag, DEFAULT_TOPIC_MAG))
    ROS_INFO("Using default imu_mag/topic_mag: %s", topic_mag.c_str());

  // Get serial port parameters
  if (!n.param<std::string>("serial_port", port, DEFAULT_SERIAL_PORT))
    ROS_INFO("Using default serialport: %s", port.c_str());

  if (!n.param<int>("baud_rate", baud, DEFAULT_SERIAL_BAUD))
    ROS_INFO("Using default baud_rate: %d", baud);




  // port, baudrate, timeout in milliseconds
  serial::Serial ser(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Serial port status:";
  if(ser.isOpen())
    cout << " Success, port " << port << " open, baud " << baud << endl;
  else {
    cout << " ERROR: Failed to open serial port " << port << endl;
    // TODO: end program or exception
    exit(0);
  }

  // Topic Advertisement and Subscription
  ros::Subscriber cmd_vel_sub = n.subscribe(topic_cmd_vel, 2, cmd_vel_callback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(topic_odom, 10);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>(topic_imu, 10);
  ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>(topic_mag, 10);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Rate loop_rate(100);

  diff_wheel_odometry odom(wheel_distance, pulses_per_m, 0, 0, 0, 0, 0, 0, 0, odom_pose_cov, odom_twist_cov);
  odom.set_odom_frame_id(odom_frame_id);
  odom.set_odom_child_frame_id(odom_child_frame_id);
  imu_mag imu(gain_acc, cov_acc, gain_gyro, cov_gyro, gain_mag, cov_mag);

  // TODO: lidar
  
  // Main LOOP  ////////////////////////////////////////////////////////////////
  while (ros::ok())
  {
    if(ser.available()){
      //ROS_INFO_STREAM("Reading from serial port");
      inputs = ser.read(ser.available());
      std::stringstream inputss(inputs);
      inputs.clear();

      while (ros::ok() && !inputss.eof()) {
        getline(inputss, message, '\n');
        remainder += message;
        //std::stringstream remainderss(remainder);
        //cout << remainder;
        if (!inputss.eof()) {
          switch (remainder[0]) {
            case MSG_IMU:
             // cout << 'i' ;//<< endl;
            index = remainder.find_first_of(';',0) + 1;
            index = remainder.find_first_of(';',index) + 1;
            d1 = atof(&remainder.c_str()[index]);
            index = remainder.find_first_of(';',index) + 1;
            d2 = atof(&remainder.c_str()[index]);
            index = remainder.find_first_of(';',index) + 1;
            d3 = atof(&remainder.c_str()[index]);
            index = remainder.find_first_of(';',index) + 1;
            d4 = atof(&remainder.c_str()[index]);
            index = remainder.find_first_of(';',index) + 1;
            d5 = atof(&remainder.c_str()[index]);
            index = remainder.find_first_of(';',index) + 1;
            d6 = atof(&remainder.c_str()[index]);
            index = remainder.find_first_of(';',index) + 1;
            d7 = atof(&remainder.c_str()[index]);
            index = remainder.find_first_of(';',index) + 1;
            d8 = atof(&remainder.c_str()[index]);
            index = remainder.find_first_of(';',index) + 1;
            d9 = atof(&remainder.c_str()[index]);
            index = remainder.find_first_of(';',index) + 1;
            imu.update(d1, d2, d3, d4, d5, d6, d7, d8, d9);
            imu_pub.publish(imu.imu_msg());
            mag_pub.publish(imu.mag_msg());
              
            break;
            case MSG_ODOMETRY:
              //cout << remainder << endl;//<< endl;
              index = remainder.find_first_of(';',0) + 1;
              index = remainder.find_first_of(';',index) + 1;
              d1 = atof(&remainder.c_str()[index]);
              index = remainder.find_first_of(';',index) + 1;
              d2 = atof(&remainder.c_str()[index]);
              //cout << d1 << "\t" << d2 << endl;
              odom.update(d1, d2);
              odom_pub.publish(odom.odom_msg());
              odom_broadcaster.sendTransform(odom.tf_msg());
            break;
            //default:
            //  while (ros::ok() && remainder)
          }
          // remainder will be cleared if the end of inputss is not reached
          // meaning remainder contains a full line (message). Otherwise the 
          // end of the message will be appended on the next itarations 
          remainder.clear();
        }
      }
      //cout << endl;
    }
    

    std::stringstream outputss;
    size_t bytes_wrote;
    outputss << "v;" << (float)(g_vel_l) << ";" << (float)(g_vel_r) << ";" << endl;
    outputs = outputss.str();
    bytes_wrote = ser.write(outputs);

    //cout << "sent: " << bytes_wrote << " bytes " << outputs << endl;
    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}


