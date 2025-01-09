from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription
import os

def generate_launch_description():
    # 声明启动参数，用于条件地启动节点
    launch_subscriber = LaunchConfiguration('launch_subscriber')

    config_file = os.path.join(
        os.path.dirname(__file__),  # 当前 launch 文件所在目录
        '../config',               # 相对于 launch 文件的路径
        'RobotConfig.yaml'              # YAML 文件名
    )

    return LaunchDescription([
        # 声明一个启动参数

        # mujoco node
        Node(
            package='mujoco_node',
            executable='mujoco_node',
            name='MujocoMsgHandler',
            output='screen'
        ),
        # robot hardware node
        Node(
            package='robot_hardware',
            executable='robot_hardware_node',
            name='RobotHardwareNode',
            output='screen'
        ),
        # main node
        Node(
            package='robot_software',
            executable='main_app',
            output='screen',
            parameters=[config_file]
        ),
        # Node(
        #     package='foxglove_bridge',
        #     executable='foxglove_bridge',
        #     name='foxglove_bridge'
        #     ,output='screen',
        #     prefix='terminator -x bash -c'
        # ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'keyboard_control', 'keyboard_publisher'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '-x', 'ros2', 'run', 'foxglove_bridge', 'foxglove_bridge'],
            output='screen'
        )
    ])
