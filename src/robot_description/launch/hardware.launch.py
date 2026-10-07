from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_yaml = (
        PathSubstitution(FindPackageShare("robot_description"))
        / "config"
        / "my_bot_controllers.yaml"
    )

    # Explicitly typed as a string rather than left for launch_ros to guess
    # via a YAML parse — the generated URDF/xacro output can contain
    # characters (in comments, mesh paths, etc.) that trip up that
    # auto-detection and raise "Unable to parse the value of parameter
    # robot_description as yaml".
    robot_description_content = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                PathSubstitution(FindPackageShare("robot_description"))
                / "urdf"
                / "my_bot.urdf.xacro",
                " ",
                "use_gazebo:=false",
                " ",
                "use_mock_hardware:=",
                LaunchConfiguration("use_mock_hardware"),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start RViz2 automatically with this launch file.",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Start robot with mock hardware mirroring command to its states.",
            ),
            # Control node — talks to the real ESP32 (or mock hardware) via
            # the DiffBotSystemHardware plugin. Run
            # `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
            # in a separate terminal before (or after) this launch file so
            # its log output stays easy to read on its own.
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                name="controller_manager",
                parameters=[
                    {
                        "robot_description": robot_description_content,
                        "update_rate": 50,
                        # Pre-declared as startup overrides (rather than left
                        # for `spawner --param-file` to set dynamically after
                        # startup) to work around a known upstream bug where
                        # controller_manager's set_parameters service rejects
                        # these as "not declared" despite
                        # allow_undeclared_parameters(true) — see
                        # ros2/rclcpp#851, ros-controls/ros2_control#3639,
                        # #3497. hardware/README.md documents this in full.
                        "joint_state_broadcaster.type": "joint_state_broadcaster/JointStateBroadcaster",
                        "joint_state_broadcaster.params_file": [controllers_yaml],
                        "diff_drive_base_controller.type": "diff_drive_controller/DiffDriveController",
                        "diff_drive_base_controller.params_file": [controllers_yaml],
                        "diff_drive_base_controller.node_options_args": [
                            "-r",
                            "~/cmd_vel:=/cmd_vel",
                        ],
                    },
                ],
                output="both",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[{"robot_description": robot_description_content}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            # Bare controller names, no --param-file/--controller-ros-args:
            # type/params_file/node_options_args are already pre-declared
            # above on controller_manager's own startup parameters, so
            # spawner never needs its (broken, see above) dynamic
            # set_parameters step and goes straight to load_controller.
            Node(
                package="controller_manager",
                executable="spawner",
                name="joint_state_broadcaster_spawner",
                arguments=["joint_state_broadcaster"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                name="diff_drive_base_controller_spawner",
                arguments=["diff_drive_base_controller"],
            ),
        ]
    )
