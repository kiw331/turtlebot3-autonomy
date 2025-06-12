from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mission_pkg_dir = get_package_share_directory('turtlebot3_autorace_mission')

    param_files = [
        os.path.join(mission_pkg_dir, 'param', 'drive_modes.yaml'),
        os.path.join(mission_pkg_dir, 'param', 'ui_texts.yaml')
    ]
        
    control_node = Node(
        package='turtlebot3_autorace_mission',
        executable='control_lane2',
        name='control_lane',
        output='screen',
        parameters=param_files,
        remappings=[
            ('/control/lane', '/detect/lane'),
            ('/control/cmd_vel', '/cmd_vel')
        ],
        prefix=['xterm -hold -e'],  # 필요 시 주석 해제
        emulate_tty=True            # 기본적으로 지원 안 하므로 주석 유지
    )
        
    gui_node = Node(
        package='turtlebot3_autorace_mission',
        executable='autorace_gui',
        name='autorace_gui',
        output='screen',
        prefix="env QT_QPA_PLATFORM_PLUGIN_PATH=''"  # Qt 플러그인 충돌 방지
    )

    return LaunchDescription([
        control_node,
        gui_node
    ])
