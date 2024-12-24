cmake_minimum_required(VERSION 3.21)



project(robot_software)

# 检查并设置C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 验证C++标准
if(CMAKE_CXX_STANDARD LESS 20)
    message(FATAL_ERROR "Requires C++20 or higher!")
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(TBB REQUIRED)
# find_package(Eigen3 3.4 REQUIRED NO_MOUDLE)

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
aux_source_directory(${CMAKE_SOURCE_DIR}/include/robot_software/third_party/quadprog Quadprog_SRC)


# add_subdirectory(${CMAKE_SOURCE_DIR}/include/robot_software/third_party/ungar)


# add main_app
add_executable(main_app ${source} ${Quadprog_SRC})
target_compile_features(main_app PUBLIC cxx_std_20)  # 确保使用 C++20 特性

set_target_properties(main_app PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
)

target_compile_definitions(main_app PRIVATE
        REQUIRE_CPP_20=$<NOT:$<CXX_COMPILER_VERSION:$<COMPILE_LANGUAGE:CXX>>>
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
        pinocchio::pinocchio
        TBB::tbb
        # ungar::ungar
        )

#add_library(lib_cpp20 SHARED src/cpp20_code.cpp) #!!!
#target_compile_features(lib_cpp20 PRIVATE cxx_std_20) #!!!

target_include_directories(main_app PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# 
install(TARGETS main_app
        DESTINATION lib/${PROJECT_NAME})

# 修改配置文件的安装路径
install(DIRECTORY 
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config
        DESTINATION share/${PROJECT_NAME}/config
)
ament_package()

