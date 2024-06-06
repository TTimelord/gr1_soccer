cmake_minimum_required(VERSION 3.16)

project(rl_s2r)

# ros depend
find_package(catkin REQUIRED
        COMPONENTS
        )

catkin_package(
        INCLUDE_DIRS
                include
        LIBRARIES
                ${PROJECT_NAME}
        CATKIN_DEPENDS
                # dynamic_reconfigure
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# your libtorch install dir
set(Torch_DIR ~/libtorch/share/cmake/Torch)  
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# add policy path
add_definitions(-DPOLICY_PATH="${CMAKE_SOURCE_DIR}/policy")

add_executable(rl_s2r src/rl_s2r.cpp)
target_link_libraries(rl_s2r "${TORCH_LIBRARIES}") 
set_property(TARGET rl_s2r PROPERTY CXX_STANDARD 14)
