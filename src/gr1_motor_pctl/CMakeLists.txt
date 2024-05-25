cmake_minimum_required(VERSION 3.16.1)
set(CMAKE_CXX_STANDARD 17)
project(gr1_motor_pctl)

# ros
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

# motor
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty)
include_directories(/eigen)

add_executable(gr1_motor_pctl
      ${PROJECT_SOURCE_DIR}/src/gr1_motor_pctl.cpp
)

target_link_libraries(gr1_motor_pctl 
                      ${PROJECT_SOURCE_DIR}/lib/libfsasdk.so 
                      pthread)
