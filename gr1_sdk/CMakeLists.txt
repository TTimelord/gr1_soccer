cmake_minimum_required(VERSION 3.10)
project(gr1sdk)

find_package(Threads REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/fsa_sdk)
include_directories(${PROJECT_SOURCE_DIR}/include/fse_sdk)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty)

include_directories(/usr/include/eigen3)


aux_source_directory(${PROJECT_SOURCE_DIR}/src/ch108IMU/ dir_imu)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/fsa_sdk/ dir_fsa)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/fse_sdk/ dir_fse)

add_library(${PROJECT_NAME} SHARED 
            ${dir_fse}
            ${dir_fsa}
            ${dir_imu}
            src/GR1HW.cpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        #${Threads_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/parallel_ankle/libParallelAnkle.so
        pthread
        )
