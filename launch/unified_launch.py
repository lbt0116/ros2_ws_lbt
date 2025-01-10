from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription
import os

def generate_launch_description():
    # 声明启动参数，用于选择启动模式
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mujoco',
        description='Choose mode: "mujoco" for simulation or "hardware" for real hardware'
    )
    mode = LaunchConfiguration('mode')

    config_file = os.path.join(
        os.path.dirname(__file__),  # 当前 launch 文件所在目录
        '../config',               # 相对于 launch 文件的路径
        'RobotConfig.yaml'              # YAML 文件名
    )

    return LaunchDescription([
        # 添加启动参数
        mode_arg,

        # mujoco node - 当 mode 为 mujoco 时启动
        Node(
            package='mujoco_node',
            executable='mujoco_node',
            name='MujocoMsgHandler',
            output='screen',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'mujoco'"]))
        ),
        # robot hardware node - 当 mode 为 hardware 时启动
        Node(
            package='robot_hardware',
            executable='robot_hardware_node',
            name='RobotHardwareNode',
            output='screen',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'hardware'"]))
        ),
        # main node
        Node(
            package='robot_software',
            executable='main_app',
            output='screen',
            parameters=[config_file]
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'keyboard_control', 'keyboard_publisher'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '-x', 'ros2', 'run', 'foxglove_bridge', 'foxglove_bridge'],
            output='screen'
        )
    ])
