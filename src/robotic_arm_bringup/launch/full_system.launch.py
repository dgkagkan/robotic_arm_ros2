from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bringup_dir = get_package_share_directory('robotic_arm_bringup')

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'gazebo.py')
        )
    )

    # 2. MoveIt - περιμένει 5 δευτερόλεπτα για να ξεκινήσει το Gazebo
    moveit = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'moveit.launch.xml')
                )
            )
        ]
    )

    # 3. Optical Action Server - περιμένει 10 δευτερόλεπτα
    optical_action_server = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='robotic_arm_commander',
                executable='optical_action_server',
                name='optical_action_server',
                output='screen',
            )
        ]
    )

    # 4. Optical Server - περιμένει 10 δευτερόλεπτα
    optical_server = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='robotic_arm_commander',
                executable='optical_server',
                name='optical_server',
                output='screen',
            )
        ]
    )

    # 5. Teleop - σε ξεχωριστό terminal, περιμένει 12 δευτερόλεπτα
    teleop = TimerAction(
        period=17.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--',
                    'bash', '-c',
                    'ros2 run robotic_arm_commander teleop; exec bash'
                ],
                output='screen',
            )
        ]
    )

    # 6. Camera debug view - σε ξεχωριστό terminal, περιμένει 13 δευτερόλεπτα
    camera_view = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'rqt_image_view', 'rqt_image_view',
                    '/camera/debug_image'
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        moveit,
        optical_action_server,
        optical_server,
        teleop,
        camera_view,
    ])