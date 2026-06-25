"""Launch the SpaceMouse Cartesian jog bridge (spacemouse_servo).

Defaults to driving ``ikt_pose_commander`` (publishes to
``ikt_pose_commander/target_pose``). Set ``target_pose_topic`` to a FZI
controller's ``target_frame`` to use the cartesian_motion_controller path
instead.

The ``spacenav`` driver is launched SEPARATELY (it is shared hardware):
``ros2 launch spacemouse spacemouse.launch.py``.

``base_frame`` and ``tip_frame`` are REQUIRED (the node refuses to start
without them).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from spacemouse_teleop.teleop_defaults import load_defaults


def generate_launch_description():
    # Defaults come from config/robot_config.yaml (spacemouse_teleop section),
    # like the other packages; CLI args still override.
    d, source = load_defaults()

    base_frame = LaunchConfiguration('base_frame')
    tip_frame = LaunchConfiguration('tip_frame')
    target_pose_topic = LaunchConfiguration('target_pose_topic')
    jog_frame = LaunchConfiguration('jog_frame')
    output_mode = LaunchConfiguration('output_mode')
    enable_commander = LaunchConfiguration('enable_commander')
    dashboard_port = LaunchConfiguration('dashboard_port')

    args = [
        DeclareLaunchArgument('base_frame', default_value=str(d['base_frame']),
                              description='TF base frame targets are expressed in (REQUIRED).'),
        DeclareLaunchArgument('tip_frame', default_value=str(d['tip_frame']),
                              description='TF end-effector frame to capture/jog (REQUIRED).'),
        DeclareLaunchArgument('target_pose_topic',
                              default_value=str(d['target_pose_topic']),
                              description='Where to publish the PoseStamped target.'),
        DeclareLaunchArgument('jog_frame', default_value=str(d['jog_frame']),
                              description="'base' (base-frame jog) or 'tool'."),
        DeclareLaunchArgument('output_mode', default_value=str(d['output_mode']),
                              description="'absolute' (target_pose) or 'delta' (jog)."),
        DeclareLaunchArgument('enable_commander',
                              default_value=str(d['enable_commander']).lower(),
                              description='Call the commander enable/disable on engage/release.'),
        DeclareLaunchArgument('dashboard_port', default_value=str(d['dashboard_port']),
                              description='If set (e.g. 8200), also launch the on/off dashboard.'),
    ]

    servo = Node(
        package='spacemouse_teleop',
        executable='servo_node',
        name='spacemouse_servo',
        output='screen',
        parameters=[{
            # CLI-overridable
            'base_frame': base_frame,
            'tip_frame': tip_frame,
            'target_pose_topic': target_pose_topic,
            'jog_frame': jog_frame,
            'output_mode': output_mode,
            'enable_commander': enable_commander,
            # the rest straight from config/robot_config.yaml (spacemouse_teleop)
            'target_delta_topic': d['target_delta_topic'],
            'input_topic': d['input_topic'],
            'joy_topic': d['joy_topic'],
            'rate_hz': d['rate_hz'],
            'linear_scale': d['linear_scale'],
            'angular_scale': d['angular_scale'],
            'deadband_lin': d['deadband_lin'],
            'deadband_ang': d['deadband_ang'],
            'max_linear_speed': d['max_linear_speed'],
            'max_angular_speed': d['max_angular_speed'],
            'input_timeout': d['input_timeout'],
            'deadman_button': d['deadman_button'],
            'deadman_mode': d['deadman_mode'],
            'button1_index': d['button1_index'],
            'button1_action': d['button1_action'],
            'speed_scales': d['speed_scales'],
            'commander_enable_srv': d['commander_enable_srv'],
            'commander_disable_srv': d['commander_disable_srv'],
            'commander_snap_srv': d['commander_snap_srv'],
        }],
    )

    # Optional on/off web dashboard (only when dashboard_port is non-empty).
    dashboard = Node(
        package='spacemouse_teleop',
        executable='dashboard_node',
        name='spacemouse_servo_dashboard',
        output='screen',
        parameters=[{
            'port': ParameterValue(dashboard_port, value_type=int),
            'servo_ns': '/spacemouse_servo',
            'twist_topic': d['input_topic'],
            'joy_topic': d['joy_topic'],
        }],
        condition=IfCondition(PythonExpression(["'", dashboard_port, "' != ''"])),
    )

    return LaunchDescription(
        args + [LogInfo(msg='[spacemouse_teleop] config: ' + source),
                servo, dashboard])
