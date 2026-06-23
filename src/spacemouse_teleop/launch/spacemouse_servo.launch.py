"""Launch the SpaceMouse Cartesian jog bridge (spacemouse_servo).

Defaults to driving ``ikt_pose_commander`` (publishes to
``ikt_pose_commander/target_pose``). Set ``target_pose_topic`` to a FZI
controller's ``target_frame`` to use the cartesian_motion_controller path
instead. Optionally also starts the ``spacenav`` driver (``launch_driver``).

``base_frame`` and ``tip_frame`` are REQUIRED (the node refuses to start
without them).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('spacemouse_teleop')
    defaults = os.path.join(pkg, 'config', 'servo_defaults.yaml')

    base_frame = LaunchConfiguration('base_frame')
    tip_frame = LaunchConfiguration('tip_frame')
    target_pose_topic = LaunchConfiguration('target_pose_topic')
    jog_frame = LaunchConfiguration('jog_frame')
    enable_commander = LaunchConfiguration('enable_commander')
    launch_driver = LaunchConfiguration('launch_driver')

    args = [
        DeclareLaunchArgument('base_frame',
                              description='TF base frame targets are expressed in (REQUIRED).'),
        DeclareLaunchArgument('tip_frame',
                              description='TF end-effector frame to capture/jog (REQUIRED).'),
        DeclareLaunchArgument('target_pose_topic',
                              default_value='ikt_pose_commander/target_pose',
                              description='Where to publish the PoseStamped target.'),
        DeclareLaunchArgument('jog_frame', default_value='base',
                              description="'base' (base-frame jog, default) or 'tool' (body-frame)."),
        DeclareLaunchArgument('enable_commander', default_value='true',
                              description='Call the commander enable/disable on engage/release.'),
        DeclareLaunchArgument('launch_driver', default_value='false',
                              description='Also start the spacenav driver.'),
    ]

    spacenav_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('spacemouse'),
            'launch', 'spacemouse.launch.py')),
        condition=IfCondition(launch_driver),
    )

    servo = Node(
        package='spacemouse_teleop',
        executable='servo_node',
        name='spacemouse_servo',
        output='screen',
        parameters=[
            defaults,
            {
                'base_frame': base_frame,
                'tip_frame': tip_frame,
                'target_pose_topic': target_pose_topic,
                'jog_frame': jog_frame,
                'enable_commander': enable_commander,
            },
        ],
    )

    return LaunchDescription(args + [spacenav_driver, servo])
