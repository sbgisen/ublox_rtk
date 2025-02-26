cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
  cmake_policy(
    SET
    CMP0048
    NEW
  )
endif(POLICY CMP0048)

project(ublox_rtk)
find_package(catkin REQUIRED)
catkin_metapackage()
