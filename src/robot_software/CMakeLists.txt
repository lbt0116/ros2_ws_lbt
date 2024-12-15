cmake_minimum_required(VERSION 3.8)
project(robot_software)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(message_filters REQUIRED)

file(GLOB_RECURSE source
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_controller/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_estimator/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_kinematics/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_dynamics/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_planning/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_utils/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_interface/*.cpp
)

add_executable(main_app ${source})
#add_executable(mujoco_node
#        src/mujoco_node.cpp
#        src/platform_ui_adapter.cc src/glfw_adapter.cc src/glfw_dispatch.cc src/simulate.cc src/lodepng.cpp
#)#        src/platform_ui_adapter.cc src/glfw_adapter.cc src/glfw_dispatch.cc src/simulate.cc
set_target_properties(main_app PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)
ament_target_dependencies(main_app
        rclcpp
        custom_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        std_msgs
        message_filters
)

target_link_libraries(main_app
        pinocchio::pinocchio)

#add_library(lib_cpp20 SHARED src/cpp20_code.cpp) #!!!
#target_compile_features(lib_cpp20 PRIVATE cxx_std_20) #!!!

target_include_directories(main_app PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS main_app
        DESTINATION lib/${PROJECT_NAME})
ament_package()
