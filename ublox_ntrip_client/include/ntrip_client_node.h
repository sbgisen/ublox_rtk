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
#ifndef NTRIP_CLIENT_NODE_H
#define NTRIP_CLIENT_NODE_H

#include <ros/ros.h>
#include <string>

class NtripClientNode
{
public:
  NtripClientNode();
  void spin();

private:
  std::string ntrip_caster_username_;
  std::string ntrip_caster_password_;
  std::string ntrip_caster_address_;
  std::string ntrip_caster_port_;
  std::string ntrip_caster_mountpoint_;
  std::string rtcm3_serial_out_;
  std::string ntrip_caster_url_;
  std::string masked_url_;
  pid_t process_pid_ = -1;

  std::string maskCredentials(const std::string& url);
  void startStr2Str();
  void shutdown();
};

#endif  // NTRIP_CLIENT_NODE_H
