cmake_minimum_required(VERSION 3.10)
project(arm_hw)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror -Wno-inconsistent-missing-override)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        arm_common
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager
        urdf
        realtime_tools
        arm_msgs
        )

catkin_package(
        INCLUDE_DIRS  include
        LIBRARIES
        CATKIN_DEPENDS roscpp arm_common hardware_interface controller_interface controller_manager urdf realtime_tools transmission_interface joint_limits_interface
        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
        src/hardware_interface.cpp
        src/control_loop.cpp
        )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
