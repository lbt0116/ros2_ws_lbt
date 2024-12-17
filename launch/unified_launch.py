from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    # 声明启动参数，用于条件地启动节点
    launch_subscriber = LaunchConfiguration('launch_subscriber')

    return LaunchDescription([
        # 声明一个启动参数


        # SetEnvironmentVariable(
        #     'RCUTILS_CONSOLE_OUTPUT_FORMAT',
        #     '[{time:%Y-%m-%d %H:%M:%S}] [{name}] [{severity}]: {message}'
        # ),

        # 发布者节点
        Node(
            package='my_publisher',
            executable='publisher_node',
            name='custom_publisher',
            output='screen'
        ),

        # mujoco node
        Node(
            package='mujoco_node',
            executable='mujoco_node',
            name='MujocoMsgHandler',
            output='screen'
        ),
        # main node
        Node(
            package='robot_software',
            executable='main_app',
            output='screen'
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[]
        )
    ])
