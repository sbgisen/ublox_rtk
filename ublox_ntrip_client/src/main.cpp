#include "ntrip_client_node.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ntrip_client_node");
  NtripClientNode node;
  node.spin();
  return 0;
}
