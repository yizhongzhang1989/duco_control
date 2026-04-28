"""Launch the Duco F/T sensor node.

Examples:
  ros2 launch duco_ft_sensor ft_sensor.launch.py
  ros2 launch duco_ft_sensor ft_sensor.launch.py port:=/dev/ttyUSB0 frame_id:=tool0
  ros2 launch duco_ft_sensor ft_sensor.launch.py tare_on_start:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud", default_value="460800"),
        DeclareLaunchArgument("frame_id", default_value="ft_sensor_link"),
        DeclareLaunchArgument("publish_rate", default_value="0.0",
                              description="0.0 = publish every frame (~960 Hz)"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("tare_on_start", default_value="false"),
        DeclareLaunchArgument("namespace", default_value=""),
    ]

    node = Node(
        package="duco_ft_sensor",
        executable="ft_sensor_node",
        name="duco_ft_sensor",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud"),
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "autostart": LaunchConfiguration("autostart"),
            "tare_on_start": LaunchConfiguration("tare_on_start"),
        }],
    )

    return LaunchDescription([*args, node])
