cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
  cmake_policy(
    SET
    CMP0048
    NEW
  )
endif(POLICY CMP0048)

project(ublox_dead_reckoning)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             sensor_msgs
             geometry_msgs
             tf2_ros
             tf2_eigen
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_eigen
)

include_directories(include ${catkin_INCLUDE_DIRS})

file(
  GLOB
  MY_SOURCES
  "src/*.cpp"
)

add_executable(ublox_dead_reckoning ${MY_SOURCES})
target_link_libraries(ublox_dead_reckoning ${catkin_LIBRARIES})
