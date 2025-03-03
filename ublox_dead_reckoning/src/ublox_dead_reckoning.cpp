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

#include <ublox_dead_reckoning.h>

UbloxDeadReckoning::UbloxDeadReckoning(ros::NodeHandle& main_nh)
  : fix_ready_(false)
  , heading_ready_(false)
  , imu_ready_(false)
  , odom_ready_(false)
  , fix_initialized_(false)
  , pubish_odom_gnss_(false)
  , publish_fix_fused_(false)
  , publish_heading_fused_(false)
  , use_imu_(false)
  , use_odom_(false)
  , parent_frame_id_("")
  , child_frame_id_("")
  , relative_orinetation_(false)
{
  readParams(main_nh);
  initSubscriber(main_nh);
  initPublisher(main_nh);
  initTimer(main_nh);
  imu_data_.header.stamp = fix_data_.header.stamp = heading_data_.header.stamp = odom_data_.header.stamp =
      initialpose_data_.header.stamp = ros::Time::now();
  heading_initial_.orientation.x = 0;
  heading_initial_.orientation.y = 0;
  heading_initial_.orientation.z = 0;
  heading_initial_.orientation.w = 1;
  odom_gnss_.header.stamp = ros::Time::now();
  odom_gnss_.pose.pose.position.x = 0;
  odom_gnss_.pose.pose.position.y = 0;
  odom_gnss_.pose.pose.position.z = 0;
  odom_gnss_.pose.pose.orientation.x = 0;
  odom_gnss_.pose.pose.orientation.y = 0;
  odom_gnss_.pose.pose.orientation.z = 0;
  odom_gnss_.pose.pose.orientation.w = 1;
  for (int i = 0; i < odom_gnss_.pose.covariance.size(); i++)
  {
    odom_gnss_.pose.covariance[i] = 0;
  }
  odom_fused_ = odom_gnss_;
  odom_smoothed_ = odom_gnss_;
}

UbloxDeadReckoning::~UbloxDeadReckoning()
{
  ros::shutdown();
}

void UbloxDeadReckoning::readParams(ros::NodeHandle& main_nh)
{
  main_nh.param<std::string>("parent_frame_id", parent_frame_id_, "gnss");
  odom_gnss_.header.frame_id = parent_frame_id_;
  odom_fused_.header.frame_id = parent_frame_id_;
  odom_smoothed_.header.frame_id = parent_frame_id_;
  main_nh.param<double>("fix_reject_covariance", fix_reject_covariance_, 1.0);
  main_nh.param<double>("fix_trust_covariance", fix_trust_covariance_, 0.1);
  main_nh.param<double>("heading_reject_covariance", heading_reject_covariance_, 1.0);
  main_nh.param<double>("heading_trust_covariance", heading_trust_covariance_, 0.01);
  main_nh.param<double>("fix_timeout", fix_timeout_, 10.0);
  main_nh.param<bool>("sensor_fusion", enable_sensor_fusion_, true);
  if (enable_sensor_fusion_)
  {
    // Wheel odometry is used for xy position and velocity,
    // and IMU is used for yaw angular velocity.
    main_nh.param<bool>("use_odom", use_odom_, true);
    main_nh.param<bool>("use_imu", use_imu_, true);
    if (!use_odom_ && !use_imu_)
    {
      ROS_ERROR("[UbloxDeadReckoning::readParams] Both odom and imu are disabled. Please enable at least one of them.");
      enable_sensor_fusion_ = false;
    }
    else
    {
      main_nh.param<bool>("relative_orinetation", relative_orinetation_, false);
      main_nh.param<bool>("pubish_odom_gnss", pubish_odom_gnss_, false);
      main_nh.param<bool>("publish_fix_fused", publish_fix_fused_, false);
      main_nh.param<bool>("publish_heading_fused", publish_heading_fused_, false);
      main_nh.param<double>("max_velocity_scale", max_velocity_scale_, 1.4);
      max_velocity_scale_ = std::max(1.0, max_velocity_scale_);
      main_nh.param<double>("min_velocity_scale", min_velocity_scale_, 0.6);
      min_velocity_scale_ = std::max(0.0, std::min(1.0, min_velocity_scale_));
      main_nh.param<double>("slip_angle_tolerance", slip_angle_tolerance_, 0.1);
      slip_angle_tolerance_ = std::max(slip_angle_tolerance_, 0.0);
    }
  }
}

void UbloxDeadReckoning::initTimer(ros::NodeHandle& main_nh)
{
  double loop_rate;
  main_nh.param<double>("rate", loop_rate, 100.0);
  timer_ = main_nh.createTimer(ros::Duration(1.0 / loop_rate), &UbloxDeadReckoning::timerCallback, this);
}

void UbloxDeadReckoning::initSubscriber(ros::NodeHandle& main_nh)
{
  ros::TransportHints transport_hints;
  transport_hints.tcpNoDelay(true);
  fix_sub_ = main_nh.subscribe("fix_in", 5, &UbloxDeadReckoning::getNavSatFixCallback, this, transport_hints);
  heading_sub_ = main_nh.subscribe("heading_in", 5, &UbloxDeadReckoning::getNavheadingCallback, this, transport_hints);
  if (use_imu_)
    imu_sub_ = main_nh.subscribe("imu_in", 5, &UbloxDeadReckoning::getImuCallback, this, transport_hints);
  if (use_odom_)
    odom_sub_ = main_nh.subscribe("odom_in", 5, &UbloxDeadReckoning::getOdomCallback, this, transport_hints);
  initialpose_sub_ =
      main_nh.subscribe("/initialpose", 5, &UbloxDeadReckoning::getInitialposeCallback, this, transport_hints);
}

void UbloxDeadReckoning::initPublisher(ros::NodeHandle& main_nh)
{
  odom_fused_pub_ = main_nh.advertise<nav_msgs::Odometry>("odom_fused", 5);
  if (publish_heading_fused_)
    heading_fused_pub_ = main_nh.advertise<sensor_msgs::Imu>("heading_fused", 5);
  if (publish_fix_fused_)
    fix_fused_pub_ = main_nh.advertise<sensor_msgs::NavSatFix>("fix_fused", 5);
  if (pubish_odom_gnss_)
    odom_gnss_pub_ = main_nh.advertise<nav_msgs::Odometry>("odom_gnss", 5);
}

void UbloxDeadReckoning::getNavSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  double fix_covariance = msg->position_covariance[0];
  // Reject fix data with large covariance
  if (fix_covariance > fix_reject_covariance_)
  {
    ROS_WARN_THROTTLE(10.0, "[UbloxDeadReckoning::getNavSatFixCallback] Covariance of fix data is too large.");
    return;
  }
  // Initialize when the first fix data received
  if (child_frame_id_ != msg->header.frame_id && !msg->header.frame_id.empty() &&
      msg->header.frame_id != parent_frame_id_)
  {
    child_frame_id_ = msg->header.frame_id;
    odom_gnss_.child_frame_id = child_frame_id_;
    odom_fused_.child_frame_id = child_frame_id_;
    odom_smoothed_.child_frame_id = child_frame_id_;
  }
  fix_ready_ = true;
  fix_data_ = *msg;
  if (!fix_initialized_)
  {
    if (!heading_ready_)
    {
      return;
    }
    // The origin of the odometry when the gnss information is first received is used as gnss origin
    ROS_INFO("[UbloxDeadReckoning::getNavSatFixCallback] Got initial fix and navheading. Start initialization.");
    tf2::Vector3 initial2robot_position(odom_fused_.pose.pose.position.x, odom_fused_.pose.pose.position.y,
                                        odom_fused_.pose.pose.position.z);
    tf2::Quaternion initial2robot_orientation(odom_fused_.pose.pose.orientation.x, odom_fused_.pose.pose.orientation.y,
                                              odom_fused_.pose.pose.orientation.z, odom_fused_.pose.pose.orientation.w);
    tf2::Quaternion utm2robot_orinetation(heading_data_.orientation.x, heading_data_.orientation.y,
                                          heading_data_.orientation.z, heading_data_.orientation.w);
    double roll, pitch, initial2robot_yaw, utm2robot_yaw;
    tf2::Matrix3x3(initial2robot_orientation).getRPY(roll, pitch, initial2robot_yaw);
    tf2::Matrix3x3(utm2robot_orinetation).getRPY(roll, pitch, utm2robot_yaw);
    double utm2initial_yaw = utm2robot_yaw - initial2robot_yaw;
    tf2::Quaternion utm2initial_orientation;
    utm2initial_orientation.setRPY(0, 0, utm2initial_yaw);
    heading_initial_ = heading_data_;
    heading_initial_.orientation.x = utm2initial_orientation.x();
    heading_initial_.orientation.y = utm2initial_orientation.y();
    heading_initial_.orientation.z = utm2initial_orientation.z();
    heading_initial_.orientation.w = utm2initial_orientation.w();
    tf2::Vector3 initial2robot_position_utm = tf2::Matrix3x3(utm2initial_orientation) * initial2robot_position;
    fix_initial_ = getFixFromTravel(-initial2robot_position_utm.getX(), -initial2robot_position_utm.getY(), fix_data_);

    // Set odom_gnss to initial position
    odom_gnss_.header.stamp = odom_fused_.header.stamp = odom_smoothed_.header.stamp = fix_data_.header.stamp;
    odom_gnss_.pose.pose.position.x = initial2robot_position_utm.getX();
    odom_gnss_.pose.pose.position.y = initial2robot_position_utm.getY();
    odom_gnss_.pose.pose.orientation.x = utm2robot_orinetation.x();
    odom_gnss_.pose.pose.orientation.y = utm2robot_orinetation.y();
    odom_gnss_.pose.pose.orientation.z = utm2robot_orinetation.z();
    odom_gnss_.pose.pose.orientation.w = utm2robot_orinetation.w();
    odom_gnss_.pose.covariance[0] = fix_initial_.position_covariance[0];
    odom_gnss_.pose.covariance[1] = fix_initial_.position_covariance[1];
    odom_gnss_.pose.covariance[6] = fix_initial_.position_covariance[3];
    odom_gnss_.pose.covariance[7] = fix_initial_.position_covariance[4];
    odom_fused_.pose.pose = odom_gnss_.pose.pose;
    odom_fused_.pose.covariance[0] = odom_gnss_.pose.covariance[0];
    odom_fused_.pose.covariance[1] = odom_gnss_.pose.covariance[1];
    odom_fused_.pose.covariance[6] = odom_gnss_.pose.covariance[6];
    odom_fused_.pose.covariance[7] = odom_gnss_.pose.covariance[7];
    odom_smoothed_ = odom_fused_;

    fix_initialized_ = true;
    ROS_INFO_STREAM("- initial\n  - fix: (" << fix_initial_.latitude << ", " << fix_initial_.longitude
                                            << ")\n  - azimuth: " << utm2initial_yaw);
    ROS_INFO_STREAM("- current\n  - fix: (" << fix_data_.latitude << ", " << fix_data_.longitude
                                            << ")\n  - azimuth: " << utm2robot_yaw);
    ROS_INFO_STREAM("[UbloxDeadReckoning::getNavSatFixCallback] Initialization is done.");
  }
  else
  {
    double travel_x, travel_y;
    getTravelFromFix(travel_x, travel_y, fix_data_, fix_initial_);
    odom_gnss_.header.stamp = odom_fused_.header.stamp = fix_data_.header.stamp;
    odom_gnss_.pose.pose.position.x = travel_x;
    odom_gnss_.pose.pose.position.y = travel_y;
    odom_gnss_.pose.pose.position.z = 0;
    odom_gnss_.pose.covariance[0] = fix_data_.position_covariance[0];
    odom_gnss_.pose.covariance[1] = fix_data_.position_covariance[1];
    odom_gnss_.pose.covariance[6] = fix_data_.position_covariance[3];
    odom_gnss_.pose.covariance[7] = fix_data_.position_covariance[4];
    odom_fused_.pose.pose.position = odom_gnss_.pose.pose.position;
    odom_fused_.pose.covariance[0] = odom_gnss_.pose.covariance[0];
    odom_fused_.pose.covariance[1] = odom_gnss_.pose.covariance[1];
    odom_fused_.pose.covariance[6] = odom_gnss_.pose.covariance[6];
    odom_fused_.pose.covariance[7] = odom_gnss_.pose.covariance[7];
  }
}

void UbloxDeadReckoning::getNavheadingCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  heading_data_ = *msg;
  double heading_covariance = heading_data_.orientation_covariance[8];
  if (heading_covariance > heading_reject_covariance_)
  {
    ROS_INFO_THROTTLE(10.0, "[UbloxDeadRecokning::getNavheadingCallback] Covariance of heading data is too large. Skip "
                            "the data.");
    return;
  }
  odom_gnss_.header.stamp = heading_data_.header.stamp;
  odom_gnss_.pose.pose.orientation = heading_data_.orientation;
  odom_gnss_.pose.covariance[35] = heading_data_.orientation_covariance[8];
  heading_ready_ = true;
  if (fix_initialized_)
  {
    odom_fused_.pose.pose.orientation = odom_gnss_.pose.pose.orientation;
    odom_fused_.pose.covariance[35] = odom_gnss_.pose.covariance[35];
  }
}

void UbloxDeadReckoning::getImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // Validate imu data
  double time_diff = std::min(1.0, (imu_data_.header.stamp - msg->header.stamp).toSec());
  imu_data_ = *msg;
  if (time_diff < 1.0e-6)
  {
    ROS_WARN_THROTTLE(1.0, "[UbloxDeadReckoning::getImuCallback] Time difference is too small.");
    return;
  }
  odom_fused_.twist.twist.angular = imu_data_.angular_velocity;
  odom_fused_.twist.covariance[35] = imu_data_.angular_velocity_covariance[8];
  imu_ready_ = true;

  tf2::Quaternion global2robot_orientation(odom_fused_.pose.pose.orientation.x, odom_fused_.pose.pose.orientation.y,
                                           odom_fused_.pose.pose.orientation.z, odom_fused_.pose.pose.orientation.w);
  double roll, pitch, global2robot_yaw;
  tf2::Matrix3x3(global2robot_orientation).getRPY(roll, pitch, global2robot_yaw);
  global2robot_yaw += time_diff * imu_data_.angular_velocity.z;
  global2robot_yaw = normalizeAngle(global2robot_yaw);
  global2robot_orientation.setRPY(0, 0, global2robot_yaw);
  odom_fused_.header.stamp = imu_data_.header.stamp;
  odom_fused_.pose.pose.orientation.x = global2robot_orientation.x();
  odom_fused_.pose.pose.orientation.y = global2robot_orientation.y();
  odom_fused_.pose.pose.orientation.z = global2robot_orientation.z();
  odom_fused_.pose.pose.orientation.w = global2robot_orientation.w();
  odom_fused_.pose.covariance[35] += imu_data_.angular_velocity_covariance[8] * time_diff;
}

void UbloxDeadReckoning::getOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Validate odom data
  double time_diff = std::min(1.0, (odom_data_.header.stamp - msg->header.stamp).toSec());
  odom_data_ = *msg;
  if (time_diff < 1.0e-6)
  {
    ROS_WARN_THROTTLE(1.0, "[UbloxDeadReckoning::getOdomCallback] Time difference is too small.");
    return;
  }
  odom_fused_.twist.twist.linear = odom_data_.twist.twist.linear;
  odom_fused_.twist.covariance[0] = odom_data_.twist.covariance[0];
  odom_fused_.twist.covariance[7] = odom_data_.twist.covariance[7];
  odom_ready_ = true;

  // Initialize child frame id
  if (odom_gnss_.child_frame_id.empty() && !msg->header.frame_id.empty() && msg->header.frame_id != parent_frame_id_)
  {
    child_frame_id_ = msg->header.frame_id;
    odom_gnss_.child_frame_id = child_frame_id_;
    odom_fused_.child_frame_id = child_frame_id_;
    odom_smoothed_.child_frame_id = child_frame_id_;
  }

  // Straight motion and covariance
  tf2::Vector3 linear_velocity_local(odom_data_.twist.twist.linear.x, odom_data_.twist.twist.linear.y, 0);
  tf2::Matrix3x3 velocity_covariance_local(
      odom_data_.twist.covariance[0], odom_data_.twist.covariance[1], odom_data_.twist.covariance[2],
      odom_data_.twist.covariance[6], odom_data_.twist.covariance[7], odom_data_.twist.covariance[8],
      odom_data_.twist.covariance[12], odom_data_.twist.covariance[13], odom_data_.twist.covariance[14]);

  tf2::Quaternion global2robot_orientation(odom_fused_.pose.pose.orientation.x, odom_fused_.pose.pose.orientation.y,
                                           odom_fused_.pose.pose.orientation.z, odom_fused_.pose.pose.orientation.w);
  tf2::Matrix3x3 global2robot_rotation_matrix(global2robot_orientation);
  tf2::Vector3 linear_velocity = global2robot_rotation_matrix * linear_velocity_local;
  tf2::Matrix3x3 velocity_covariance =
      global2robot_rotation_matrix * velocity_covariance_local * global2robot_rotation_matrix.transpose();

  // Variation due to current angle error
  double velocity_length = linear_velocity_local.length();
  double velocity_direction_yaw = atan2(linear_velocity.y(), linear_velocity.x());
  tf2::Quaternion velocity_direction_orientation;
  velocity_direction_orientation.setRPY(0, 0, velocity_direction_yaw);
  tf2::Matrix3x3 velocity_direction_rotation_matrix(velocity_direction_orientation);
  tf2::Matrix3x3 slip_covariance_local(0, 0, 0, 0, odom_fused_.pose.covariance[35] * velocity_length * time_diff, 0, 0,
                                       0, 0);
  tf2::Matrix3x3 slip_covariance =
      velocity_direction_rotation_matrix * slip_covariance_local * velocity_direction_rotation_matrix.transpose();

  // Update position and covariance
  odom_fused_.header.stamp = odom_data_.header.stamp;
  odom_fused_.pose.pose.position.x += linear_velocity.x() * time_diff;
  odom_fused_.pose.pose.position.y += linear_velocity.y() * time_diff;
  odom_fused_.pose.covariance[0] += (velocity_covariance[0][0] * time_diff + slip_covariance[0][0]);
  odom_fused_.pose.covariance[1] += (velocity_covariance[0][1] * time_diff + slip_covariance[0][1]);
  odom_fused_.pose.covariance[6] += (velocity_covariance[1][0] * time_diff + slip_covariance[1][0]);
  odom_fused_.pose.covariance[7] += (velocity_covariance[1][1] * time_diff + slip_covariance[1][1]);

  if (!use_imu_)
  {
    odom_fused_.twist.twist.angular = odom_data_.twist.twist.angular;
    double roll, pitch, global2robot_yaw;
    tf2::Matrix3x3(global2robot_orientation).getRPY(roll, pitch, global2robot_yaw);
    global2robot_yaw += time_diff * odom_data_.twist.twist.angular.z;
    global2robot_yaw = normalizeAngle(global2robot_yaw);
    global2robot_orientation.setRPY(0, 0, global2robot_yaw);
    odom_fused_.pose.pose.orientation.x = global2robot_orientation.x();
    odom_fused_.pose.pose.orientation.y = global2robot_orientation.y();
    odom_fused_.pose.pose.orientation.z = global2robot_orientation.z();
    odom_fused_.pose.pose.orientation.w = global2robot_orientation.w();
    odom_fused_.pose.covariance[35] += odom_data_.twist.covariance[35] * time_diff;
  }
}

void UbloxDeadReckoning::getInitialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if (child_frame_id_.empty())
  {
    ROS_ERROR("[UbloxDeadReckoning::getInitialposeCallback] child_frame_id_ is not initialized.");
    return;
  }
  timer_.stop();
  fix_ready_ = heading_ready_ = imu_ready_ = odom_ready_ = false;
  odom_gnss_.pose.pose.position.x = 0;
  odom_gnss_.pose.pose.position.y = 0;
  odom_gnss_.pose.pose.position.z = 0;
  odom_gnss_.pose.pose.orientation = odom_fused_.pose.pose.orientation;
  // Initialize gnss origin onto rviz global frame
  tf2::Vector3 global2robot_position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  tf2::Quaternion global2robot_orientation(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                           msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Vector3 initial2robot_position_utm(odom_fused_.pose.pose.position.x, odom_fused_.pose.pose.position.y,
                                          odom_fused_.pose.pose.position.z);
  tf2::Quaternion utm2robot_orinetation(odom_fused_.pose.pose.orientation.x, odom_fused_.pose.pose.orientation.y,
                                        odom_fused_.pose.pose.orientation.z, odom_fused_.pose.pose.orientation.w);

  double roll, pitch, utm2robot_yaw, global2robot_yaw;
  tf2::Matrix3x3(utm2robot_orinetation).getRPY(roll, pitch, utm2robot_yaw);
  tf2::Matrix3x3(global2robot_orientation).getRPY(roll, pitch, global2robot_yaw);
  double utm2global_yaw = utm2robot_yaw - global2robot_yaw;
  tf2::Quaternion utm2global_orientation;
  utm2global_orientation.setRPY(0, 0, utm2global_yaw);

  tf2::Vector3 robot2global_position = tf2::Matrix3x3(global2robot_orientation).inverse() * -global2robot_position;
  tf2::Vector3 utm2global_position =
      initial2robot_position_utm + (tf2::Matrix3x3(utm2robot_orinetation) * robot2global_position);

  tf2::Vector3 global2robot_position_utm = tf2::Matrix3x3(utm2global_orientation) * global2robot_position;
  fix_initial_ = getFixFromTravel(odom_fused_.pose.pose.position.x - global2robot_position_utm.getX(),
                                  odom_fused_.pose.pose.position.y - global2robot_position_utm.getY(), fix_initial_);
  odom_gnss_.header.stamp = odom_fused_.header.stamp = msg->header.stamp;
  heading_initial_.orientation.x = utm2global_orientation.x();
  heading_initial_.orientation.y = utm2global_orientation.y();
  heading_initial_.orientation.z = utm2global_orientation.z();
  heading_initial_.orientation.w = utm2global_orientation.w();
  odom_gnss_.pose.pose.position.x = global2robot_position_utm.getX();
  odom_gnss_.pose.pose.position.y = global2robot_position_utm.getY();
  odom_fused_.pose.pose = odom_gnss_.pose.pose;
  odom_smoothed_ = odom_fused_;
  timer_.start();
}

void UbloxDeadReckoning::timerCallback(const ros::TimerEvent& /*event*/)
{
  if (fix_initialized_)
  {
    double roll, pitch, yaw;
    tf2::Quaternion q_initial(heading_initial_.orientation.x, heading_initial_.orientation.y,
                              heading_initial_.orientation.z, heading_initial_.orientation.w);
    tf2::Matrix3x3(q_initial).getRPY(roll, pitch, yaw);
    double yaw_initial = yaw;
    if (enable_sensor_fusion_ && (fix_ready_ || heading_ready_ || imu_ready_ || odom_ready_))
    {
      updateSmoothOdom();
      if (relative_orinetation_)
        odom_fused_pub_.publish(rotateOdom(odom_smoothed_, -yaw_initial));
      else
        odom_fused_pub_.publish(odom_smoothed_);
      if (pubish_odom_gnss_ && fix_ready_ && heading_ready_)
      {
        if (relative_orinetation_)
          odom_gnss_pub_.publish(rotateOdom(odom_gnss_, -yaw_initial));
        else
          odom_gnss_pub_.publish(odom_gnss_);
        fix_ready_ = heading_ready_ = imu_ready_ = odom_ready_ = false;
      }
    }
    else if (!enable_sensor_fusion_ && fix_ready_ && heading_ready_)
    {
      if (relative_orinetation_)
        odom_fused_pub_.publish(rotateOdom(odom_gnss_, -yaw_initial));
      else
        odom_fused_pub_.publish(odom_gnss_);
      fix_ready_ = heading_ready_ = imu_ready_ = odom_ready_ = false;
    }
    else
    {
      if ((ros::Time::now() - fix_data_.header.stamp).toSec() > fix_timeout_)
      {
        ROS_WARN_THROTTLE(10.0, "[UbloxDeadReckoning::timerCallback] Fix data timeout.");
        fix_ready_ = heading_ready_ = imu_ready_ = odom_ready_ = false;
      }
    }
  }
  else
  {
    ROS_WARN_THROTTLE(10.0, "[UbloxDeadReckoning::timerCallback] Waiting for initialize...");
    if (!child_frame_id_.empty())
    {
      odom_fused_pub_.publish(odom_fused_);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "[UbloxDeadReckoning::timerCallback] No data found. Skip publishing.");
    }
  }
}

void UbloxDeadReckoning::getTravelFromFix(double& travel_x, double& travel_y, const sensor_msgs::NavSatFix& fix,
                                          const sensor_msgs::NavSatFix& fix_initial)
{
  // Get travel distance using haversine formula
  static const double EARTHS_RADIUS = 6378137.0;
  double lat_rad = deg2Rad(fix.latitude);
  double lat_init_rad = deg2Rad(fix_initial.latitude);
  double lon_rad = deg2Rad(fix.longitude);
  double lon_init_rad = deg2Rad(fix_initial.longitude);
  double lat_delta = lat_rad - lat_init_rad;
  double lon_delta = lon_rad - lon_init_rad;
  double a = pow(sin(lat_delta / 2.0), 2.0) +
             cos(lat_rad) * cos(lat_init_rad) *
                 pow(sin(lon_delta / 2.0), 2.0);  // square of half the chord length between the points
  double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));   // Angle
  double distance = EARTHS_RADIUS * c;            // Distance
  // Get direction using spherical coordinates
  Eigen::Vector3f fix_cartesian(0, 0, 1.0);
  Eigen::Vector3f x_axis(1.0, 0, 0), y_axis(0, 1.0, 0);
  fix_cartesian =
      Eigen::AngleAxisf(lon_rad, y_axis).matrix() * Eigen::AngleAxisf(-lat_rad, x_axis).matrix() * fix_cartesian;
  Eigen::Matrix3f rotation_matrix =
      Eigen::AngleAxisf(lat_init_rad, x_axis).matrix() * Eigen::AngleAxisf(-lon_init_rad, y_axis).matrix();
  Eigen::Vector3f fix_relative = rotation_matrix * fix_cartesian;
  double yaw = atan2(fix_relative(1), fix_relative(0));
  travel_x = distance * cos(yaw);
  travel_y = distance * sin(yaw);
}

sensor_msgs::NavSatFix UbloxDeadReckoning::getFixFromTravel(const double travel_x, const double travel_y,
                                                            const sensor_msgs::NavSatFix& fix_initial)
{
  sensor_msgs::NavSatFix fix = fix_initial;
  static const double EARTHS_RADIUS = 6378137.0;
  double latitude_delta = travel_y / EARTHS_RADIUS;
  double longitude_delta = travel_x / (EARTHS_RADIUS * cos(deg2Rad(fix_initial.latitude)));
  fix.latitude = normalizeLatitude(rad2Deg(latitude_delta) + fix_initial.latitude);
  fix.longitude = normalizeLongitude(rad2Deg(longitude_delta) + fix_initial.longitude);
  return fix;
}

double UbloxDeadReckoning::deg2Rad(const double degree)
{
  return degree * M_PI / 180.0;
}

double UbloxDeadReckoning::rad2Deg(const double rad)
{
  return rad * 180.0 / M_PI;
}

double UbloxDeadReckoning::normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}

double UbloxDeadReckoning::normalizeLatitude(double degree)
{
  while (degree > 90.0)
    degree -= 180.0;
  while (degree < -90.0)
    degree += 180.0;
  return degree;
}

double UbloxDeadReckoning::normalizeLongitude(double degree)
{
  while (degree > 180.0)
    degree -= 360.0;
  while (degree < -180.0)
    degree += 360.0;
  return degree;
}

nav_msgs::Odometry UbloxDeadReckoning::rotateOdom(const nav_msgs::Odometry& odom_in, const double yaw_in)
{
  nav_msgs::Odometry odom_fused = odom_in;
  tf2::Matrix3x3 m_yaw;
  m_yaw.setRPY(0, 0, yaw_in);
  tf2::Vector3 pos_vec(odom_in.pose.pose.position.x, odom_in.pose.pose.position.y, odom_in.pose.pose.position.z);
  tf2::Matrix3x3 pos_cov(odom_in.pose.covariance[0], odom_in.pose.covariance[1], odom_in.pose.covariance[2],
                         odom_in.pose.covariance[6], odom_in.pose.covariance[7], odom_in.pose.covariance[8],
                         odom_in.pose.covariance[12], odom_in.pose.covariance[13], odom_in.pose.covariance[14]);
  tf2::Vector3 pos_rot = m_yaw * pos_vec;
  tf2::Matrix3x3 pos_cov_rot = m_yaw * pos_cov * m_yaw.transpose();
  odom_fused.pose.pose.position.x = pos_rot.x();
  odom_fused.pose.pose.position.y = pos_rot.y();
  odom_fused.pose.pose.position.z = pos_rot.z();
  odom_fused.pose.covariance[0] = pos_cov_rot[0][0];
  odom_fused.pose.covariance[1] = pos_cov_rot[0][1];
  odom_fused.pose.covariance[6] = pos_cov_rot[1][0];
  odom_fused.pose.covariance[7] = pos_cov_rot[1][1];
  double roll, pitch, yaw;
  tf2::Quaternion global2robot_orientation(odom_in.pose.pose.orientation.x, odom_in.pose.pose.orientation.y,
                                           odom_in.pose.pose.orientation.z, odom_in.pose.pose.orientation.w);
  tf2::Matrix3x3(global2robot_orientation).getRPY(roll, pitch, yaw);
  yaw += yaw_in;
  yaw = normalizeAngle(yaw);
  global2robot_orientation.setRPY(0, 0, yaw);
  odom_fused.pose.pose.orientation.x = global2robot_orientation.x();
  odom_fused.pose.pose.orientation.y = global2robot_orientation.y();
  odom_fused.pose.pose.orientation.z = global2robot_orientation.z();
  odom_fused.pose.pose.orientation.w = global2robot_orientation.w();
  return odom_fused;
}

void UbloxDeadReckoning::updateSmoothOdom()
{
  // Obtain the current timestamp and compute the time difference.
  // The time difference is constrained between 0 and 1 second to prevent unrealistic estimation jumps.
  ros::Time current_time = ros::Time::now();
  double time_diff = std::max(0.0, std::min(1.0, (current_time - odom_smoothed_.header.stamp).toSec()));

  // Extract the current position and orientation from the smoothed odometry data.
  tf2::Vector3 current_position(odom_smoothed_.pose.pose.position.x, odom_smoothed_.pose.pose.position.y, 0);
  tf2::Quaternion current_orientation(odom_smoothed_.pose.pose.orientation.x, odom_smoothed_.pose.pose.orientation.y,
                                      odom_smoothed_.pose.pose.orientation.z, odom_smoothed_.pose.pose.orientation.w);
  double roll, pitch, current_yaw;
  tf2::Matrix3x3(current_orientation).getRPY(roll, pitch, current_yaw);

  // Predict movement based on velocity data from wheel odometry and IMU.
  tf2::Vector3 velocity_local(odom_fused_.twist.twist.linear.x, odom_fused_.twist.twist.linear.y, 0);
  tf2::Vector3 predicted_travel = tf2::Matrix3x3(current_orientation) * velocity_local * time_diff;
  double predicted_yaw_change = normalizeAngle(odom_fused_.twist.twist.angular.z * time_diff);

  // Retrieve GNSS-based position and orientation measurements.
  tf2::Vector3 measured_position(odom_fused_.pose.pose.position.x, odom_fused_.pose.pose.position.y, 0);
  tf2::Quaternion measured_orientation(heading_data_.orientation.x, heading_data_.orientation.y,
                                       heading_data_.orientation.z, heading_data_.orientation.w);
  double measured_yaw;
  tf2::Matrix3x3(measured_orientation).getRPY(roll, pitch, measured_yaw);
  double measured_yaw_change = normalizeAngle(measured_yaw - current_yaw);
  tf2::Vector3 measured_travel = measured_position - current_position;

  // Compute trust factors based on measurement reliability.
  // These factors are derived from covariance values to balance GNSS accuracy and odometry consistency.
  double linear_trust =
      fix_ready_ ? (fix_trust_covariance_ / (fix_trust_covariance_ + odom_fused_.pose.covariance[0])) : 0.0;
  double yaw_trust = heading_ready_ ?
                         (heading_trust_covariance_ / (heading_trust_covariance_ + odom_fused_.pose.covariance[35])) :
                         0.0;

  // Compute the corrected movement by blending predicted and measured values
  // using their respective trust factors.
  tf2::Vector3 corrected_travel = predicted_travel + (measured_travel - predicted_travel) * linear_trust;
  double corrected_yaw_change =
      predicted_yaw_change + normalizeAngle(measured_yaw_change - predicted_yaw_change) * yaw_trust;

  // Apply velocity-dependent constraints to prevent excessive corrections that may interfere with path planning.
  double max_yaw_change = odom_fused_.twist.twist.angular.z * max_velocity_scale_ * time_diff;
  double min_yaw_change = odom_fused_.twist.twist.angular.z * min_velocity_scale_ * time_diff;
  double limited_yaw_change = std::max(min_yaw_change, std::min(max_yaw_change, corrected_yaw_change));
  double limited_yaw = normalizeAngle(current_yaw + limited_yaw_change);

  double max_travel_length = velocity_local.length() * time_diff * max_velocity_scale_;
  double min_travel_length = velocity_local.length() * time_diff * min_velocity_scale_;
  double travel_length = std::max(min_travel_length, std::min(max_travel_length, corrected_travel.length()));

  // Restrict travel direction to minimize deviations from the expected trajectory.
  tf2::Matrix3x3 rotation_matrix(current_orientation);
  tf2::Vector3 local_travel = rotation_matrix.inverse() * corrected_travel;
  double travel_direction_local = atan2(local_travel.y(), local_travel.x());
  double velocity_direction_local =
      normalizeAngle(atan2(velocity_local.y(), velocity_local.x()) + limited_yaw_change * 0.5);
  double direction_offset = normalizeAngle(travel_direction_local - velocity_direction_local);

  // If the travel direction deviates more than 90 degrees from the velocity direction,
  // suppress movement correction to prevent unintended lateral displacement.
  if (fabs(direction_offset) > M_PI * 0.5)
    max_travel_length = 0.0;

  double limited_direction_offset =
      std::max(limited_yaw_change * 0.5 - fabs(slip_angle_tolerance_),
               std::min(limited_yaw_change * 0.5 + fabs(slip_angle_tolerance_), direction_offset));

  double limited_travel_direction = velocity_direction_local + limited_direction_offset;
  tf2::Vector3 limited_travel_local(travel_length * cos(limited_travel_direction),
                                    travel_length * sin(limited_travel_direction), 0);
  tf2::Vector3 limited_travel = rotation_matrix * limited_travel_local;

  // Update the odometry data with the constrained movement values.
  nav_msgs::Odometry updated_odom = odom_fused_;
  updated_odom.header.stamp = current_time;
  tf2::Quaternion limited_orientation;
  limited_orientation.setRPY(0, 0, limited_yaw);
  updated_odom.pose.pose.position.x = odom_smoothed_.pose.pose.position.x + limited_travel.x();
  updated_odom.pose.pose.position.y = odom_smoothed_.pose.pose.position.y + limited_travel.y();
  updated_odom.pose.pose.orientation.x = limited_orientation.x();
  updated_odom.pose.pose.orientation.y = limited_orientation.y();
  updated_odom.pose.pose.orientation.z = limited_orientation.z();
  updated_odom.pose.pose.orientation.w = limited_orientation.w();

  // Adjust covariance values to account for estimation errors.
  tf2::Vector3 error_linear = limited_travel - measured_travel;
  double error_yaw = normalizeAngle(limited_yaw_change - measured_yaw_change);
  updated_odom.pose.covariance[35] += fabs(error_yaw);

  tf2::Matrix3x3 error_cov_local(fabs(error_linear.x()), 0, 0, 0, fabs(error_linear.y()), 0, 0, 0, 0);
  tf2::Matrix3x3 error_cov =
      tf2::Matrix3x3(limited_orientation) * error_cov_local * tf2::Matrix3x3(limited_orientation).transpose();
  updated_odom.pose.covariance[0] += error_cov[0][0];
  updated_odom.pose.covariance[1] += error_cov[0][1];
  updated_odom.pose.covariance[6] += error_cov[1][0];
  updated_odom.pose.covariance[7] += error_cov[1][1];

  odom_smoothed_ = updated_odom;

  // Publish the fused heading if enabled.
  if (publish_heading_fused_ && fix_initialized_)
  {
    sensor_msgs::Imu heading_fused = imu_data_;
    heading_fused.orientation = odom_smoothed_.pose.pose.orientation;
    heading_fused.orientation_covariance[8] = odom_smoothed_.pose.covariance[35];
    heading_fused_pub_.publish(heading_fused);
  }

  // Publish the fused position if enabled.
  if (publish_fix_fused_ && fix_initialized_)
  {
    sensor_msgs::NavSatFix fix_smooth =
        getFixFromTravel(odom_smoothed_.pose.pose.position.x, odom_smoothed_.pose.pose.position.y, fix_initial_);
    fix_smooth.header.stamp = odom_smoothed_.header.stamp;
    fix_fused_pub_.publish(fix_smooth);
  }
}
