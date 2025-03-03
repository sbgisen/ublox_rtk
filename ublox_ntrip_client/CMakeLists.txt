cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
  cmake_policy(
    SET
    CMP0048
    NEW
  )
endif(POLICY CMP0048)

project(ublox_ntrip_client)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})

file(
  GLOB
  MY_SOURCES
  "src/*.cpp"
)

add_executable(ntrip_client_node ${MY_SOURCES})
target_link_libraries(ntrip_client_node ${catkin_LIBRARIES})
