cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(test-libtorch)

set(Torch_DIR ~/libtorch/share/cmake/Torch)  
# set(Torch_DIR ${PROJECT_SOURCE_DIR}/libtorch/share/cmake/Torch) 

find_package(Torch REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

#main.cpp exe
add_executable(test-libtorch src/test.cpp)

#link libtorch .a .so
target_link_libraries(test-libtorch "${TORCH_LIBRARIES}")

#
set_property(TARGET test-libtorch PROPERTY CXX_STANDARD 14)
