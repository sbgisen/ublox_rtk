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
#include "ntrip_client_node.h"
#include <cstdlib>
#include <vector>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

NtripClientNode::NtripClientNode()
{
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("ntrip_caster_username", ntrip_caster_username_, "username");
  private_nh.param<std::string>("ntrip_caster_password", ntrip_caster_password_, "password");
  private_nh.param<std::string>("ntrip_caster_address", ntrip_caster_address_, "127.0.0.1");
  private_nh.param<std::string>("ntrip_caster_port", ntrip_caster_port_, "2101");
  private_nh.param<std::string>("ntrip_caster_mountpoint", ntrip_caster_mountpoint_, "mountpoint");
  private_nh.param<std::string>("rtcm3_serial_out", rtcm3_serial_out_, "ttyUblox");

  private_nh.param<std::string>("ntrip_caster_url", ntrip_caster_url_,
                                ntrip_caster_username_ + ":" + ntrip_caster_password_ + "@" + ntrip_caster_address_ +
                                    ":" + ntrip_caster_port_ + "/" + ntrip_caster_mountpoint_);

  masked_url_ = maskCredentials(ntrip_caster_url_);

  ROS_INFO("Ntrip Caster URL: %s", masked_url_.c_str());
  ROS_INFO("Output Serial Port: %s", rtcm3_serial_out_.c_str());

  startStr2Str();
}

std::string NtripClientNode::maskCredentials(const std::string& url)
{
  size_t at_pos = url.find('@');
  if (at_pos != std::string::npos)
  {
    return "<username>:<password>" + url.substr(at_pos);
  }
  return url;
}

void NtripClientNode::startStr2Str()
{
  std::vector<std::string> args = { "str2str", "-in", "ntrip://" + ntrip_caster_url_, "-out",
                                    "serial://" + rtcm3_serial_out_ };
  std::vector<char*> c_args;

  for (auto& arg : args)
  {
    c_args.push_back(const_cast<char*>(arg.c_str()));
  }
  c_args.push_back(nullptr);

  ROS_INFO("Executing command: str2str -in ntrip://%s -out serial://%s", masked_url_.c_str(),
           rtcm3_serial_out_.c_str());

  process_pid_ = fork();
  if (process_pid_ == 0)
  {
    execvp(c_args[0], c_args.data());
    ROS_ERROR("Failed to start str2str process.");
    exit(EXIT_FAILURE);
  }
  else if (process_pid_ < 0)
  {
    ROS_ERROR("Failed to fork process for str2str.");
  }
  else
  {
    ROS_INFO("str2str process started successfully.");
  }
}

void NtripClientNode::spin()
{
  ros::Rate rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  shutdown();
}

void NtripClientNode::shutdown()
{
  if (process_pid_ > 0)
  {
    ROS_INFO("Shutting down str2str process...");
    kill(process_pid_, SIGTERM);
    waitpid(process_pid_, nullptr, 0);
    ROS_INFO("str2str process terminated.");
  }
}
