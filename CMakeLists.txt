cmake_minimum_required(VERSION 2.8.3)
project(ublox_driver)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "-std=c++11 ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  gnss_comm
)

catkin_package(
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
  ${PROJECT_SOURCE_DIR}/third_party/
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/ublox_driver.cpp 
                               ${PROJECT_SOURCE_DIR}/src/ublox_message_processor.cpp
                               ${PROJECT_SOURCE_DIR}/src/file_loader.cpp
                               ${PROJECT_SOURCE_DIR}/src/file_dumper.cpp
                               ${PROJECT_SOURCE_DIR}/src/serial_handler.cpp
                               ${PROJECT_SOURCE_DIR}/src/socket_handler.cpp
                               ${PROJECT_SOURCE_DIR}/third_party/yaml/Yaml.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(sync_system_time ${PROJECT_SOURCE_DIR}/src/sync_system_time.cpp)
target_link_libraries(sync_system_time ${catkin_LIBRARIES})