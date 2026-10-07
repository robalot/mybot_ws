import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Gazebo resolves the xacro's package://robot_description/meshes/... URIs
    # as model://robot_description/meshes/..., which requires the *parent* of
    # the installed "robot_description" share dir to be on the resource path.
    robot_description_share_parent = os.path.dirname(
        get_package_share_directory("robot_description")
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
                "use_gazebo:=true",
                " ",
                "use_mock_hardware:=false",
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
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                    os.pathsep,
                    robot_description_share_parent,
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathSubstitution(FindPackageShare("ros_gz_sim"))
                    / "launch"
                    / "gz_sim.launch.py"
                ),
                launch_arguments={"gz_args": "-r -v 3 empty.sdf"}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="clock_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="both",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[{"robot_description": robot_description_content}],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_my_bot",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-name",
                    "my_bot",
                    "-allow_renaming",
                    "true",
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            # gz_ros2_control's GazeboSimROS2ControlPlugin spawns controller_manager
            # inside the Gazebo process once the plugin loads — delay the spawners
            # a couple seconds after the robot is created to give it time to come up.
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        name="joint_state_broadcaster_spawner",
                        arguments=[
                            "joint_state_broadcaster",
                            "--param-file",
                            PathSubstitution(FindPackageShare("robot_description"))
                            / "config"
                            / "my_bot_controllers.yaml",
                        ],
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        name="diff_drive_base_controller_spawner",
                        arguments=[
                            "diff_drive_base_controller",
                            "--param-file",
                            PathSubstitution(FindPackageShare("robot_description"))
                            / "config"
                            / "my_bot_controllers.yaml",
                            "--controller-ros-args",
                            "-r ~/cmd_vel:=/cmd_vel",
                        ],
                    ),
                ],
            ),
        ]
    )
