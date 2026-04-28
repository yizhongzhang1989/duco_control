"""Launch the web-based F/T sensor dashboard.

Open the printed URL in any browser on the LAN.

Examples:
  ros2 launch ft_sensor_dashboard dashboard.launch.py
  ros2 launch ft_sensor_dashboard dashboard.launch.py port:=8081
  ros2 launch ft_sensor_dashboard dashboard.launch.py topic:=/my_other_sensor/wrench
  ros2 launch ft_sensor_dashboard dashboard.launch.py window_seconds:=30.0 push_rate:=20.0
  ros2 launch ft_sensor_dashboard dashboard.launch.py reliability:=reliable
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("topic", default_value="/duco_ft_sensor/wrench"),
        DeclareLaunchArgument("host", default_value="0.0.0.0",
                              description="HTTP bind address; 0.0.0.0 = LAN-visible"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("window_seconds", default_value="10.0"),
        DeclareLaunchArgument("push_rate", default_value="30.0",
                              description="SSE updates per second"),
        DeclareLaunchArgument("reliability", default_value="best_effort",
                              description="best_effort or reliable"),
        DeclareLaunchArgument("title", default_value=""),
    ]

    node = Node(
        package="ft_sensor_dashboard",
        executable="dashboard",
        name="ft_sensor_dashboard",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "topic": LaunchConfiguration("topic"),
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "window_seconds": LaunchConfiguration("window_seconds"),
            "push_rate": LaunchConfiguration("push_rate"),
            "reliability": LaunchConfiguration("reliability"),
            "title": LaunchConfiguration("title"),
        }],
    )

    return LaunchDescription([*args, node])
