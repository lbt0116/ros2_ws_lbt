cmake_minimum_required(VERSION 3.8)
project(mujoco_node)

# 设置 C++20 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 明确指定使用 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_PREFIX_PATH "src/mujoco_node/cmake")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mujoco REQUIRED)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)

add_executable(mujoco_node ${source})
target_compile_features(mujoco_node PUBLIC cxx_std_20)  # 确保使用 C++20 特性

ament_target_dependencies(mujoco_node rclcpp custom_msgs sensor_msgs)
target_link_libraries(mujoco_node mujoco glfw)

target_include_directories(mujoco_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mujoco_node>
    $<INSTALL_INTERFACE:include/mujoco_node>
)

install(TARGETS mujoco_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
