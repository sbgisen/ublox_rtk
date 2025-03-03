/*
 * Copyright (c) 2025, SoftBank corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef UBLOX_DEAD_RECKONING_H
#define UBLOX_DEAD_RECKONING_H

#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

class UbloxDeadReckoning
{
public:
  UbloxDeadReckoning(ros::NodeHandle& main_nh);
  ~UbloxDeadReckoning();

private:
  void readParams(ros::NodeHandle& main_nh);
  void initTimer(ros::NodeHandle& main_nh);
  void initPublisher(ros::NodeHandle& main_nh);
  void initSubscriber(ros::NodeHandle& main_nh);

  void getNavSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void getNavheadingCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void getImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void getOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void getInitialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);

  void getTravelFromFix(double& travel_x, double& travel_y, const sensor_msgs::NavSatFix& fix,
                        const sensor_msgs::NavSatFix& fix_initial);
  sensor_msgs::NavSatFix getFixFromTravel(const double travel_x, const double travel_y,
                                          const sensor_msgs::NavSatFix& fix_initial);

  double deg2Rad(const double degree);
  double rad2Deg(const double rad);
  double normalizeAngle(double angle);
  double normalizeLatitude(double degree);
  double normalizeLongitude(double degree);
  nav_msgs::Odometry rotateOdom(const nav_msgs::Odometry& odom_in, const double yaw_in);
  void updateSmoothOdom();

  // ROS Objects
  ros::NodeHandle nh_;
  ros::Publisher odom_fused_pub_;
  ros::Publisher odom_gnss_pub_;
  ros::Publisher heading_fused_pub_;
  ros::Publisher fix_fused_pub_;
  ros::Subscriber fix_sub_;
  ros::Subscriber heading_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber initialpose_sub_;
  ros::Timer timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS Params
  bool enable_sensor_fusion_;
  bool use_odom_;
  bool use_imu_;
  bool relative_orinetation_;

  bool publish_heading_fused_;
  bool publish_fix_fused_;
  bool pubish_odom_gnss_;

  double fix_reject_covariance_;
  double fix_trust_covariance_;
  double heading_reject_covariance_;
  double heading_trust_covariance_;
  double fix_timeout_;
  std::string parent_frame_id_;
  std::string child_frame_id_;
  // Smoother
  double max_velocity_scale_;
  double min_velocity_scale_;
  double slip_angle_tolerance_;

  // Internal variables
  bool fix_initialized_;
  bool fix_ready_;
  bool heading_ready_;
  bool imu_ready_;
  bool odom_ready_;

  sensor_msgs::NavSatFix fix_initial_;
  sensor_msgs::NavSatFix fix_data_;

  sensor_msgs::Imu heading_initial_;
  sensor_msgs::Imu heading_data_;

  sensor_msgs::Imu imu_data_;
  nav_msgs::Odometry odom_data_;
  geometry_msgs::PoseWithCovarianceStamped initialpose_data_;

  nav_msgs::Odometry odom_gnss_;
  nav_msgs::Odometry odom_fused_;
  nav_msgs::Odometry odom_smoothed_;
};

#endif  // UBLOX_DEAD_RECKONING_H
