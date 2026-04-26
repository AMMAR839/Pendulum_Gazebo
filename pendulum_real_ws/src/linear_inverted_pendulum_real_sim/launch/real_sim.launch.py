import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("linear_inverted_pendulum_real_sim")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    robot_description_file = os.path.join(
        pkg_share, "urdf", "linear_inverted_pendulum_real.urdf.xacro"
    )
    robot_description_xml = xacro.process_file(robot_description_file).toxml()

    gz_args = LaunchConfiguration("gz_args")
    gz_partition = LaunchConfiguration("gz_partition")
    serial_link = LaunchConfiguration("serial_link")
    status_rate_hz = LaunchConfiguration("status_rate_hz")
    enable_serial_bridge = LaunchConfiguration("enable_serial_bridge")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_xml,
                "use_sim_time": True,
            }
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "name": "linear_inverted_pendulum",
                "topic": "robot_description",
                "allow_renaming": False,
            }
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/empty/model/linear_inverted_pendulum/joint_state"
            "@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/model/linear_inverted_pendulum/joint/cart_slider/cmd_force"
            "@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/linear_inverted_pendulum/joint/pendulum_hinge/cmd_force"
            "@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        remappings=[
            (
                "/world/empty/model/linear_inverted_pendulum/joint_state",
                "joint_states",
            ),
            (
                "/model/linear_inverted_pendulum/joint/cart_slider/cmd_force",
                "/pendulum/cart_force_cmd",
            ),
            (
                "/model/linear_inverted_pendulum/joint/pendulum_hinge/cmd_force",
                "/pendulum/hinge_assist_force_cmd",
            ),
        ],
    )

    serial_bridge = Node(
        package="linear_inverted_pendulum_real_sim",
        executable="real_serial_bridge",
        name="pendulum_real_serial_bridge",
        output="screen",
        condition=IfCondition(enable_serial_bridge),
        parameters=[
            {
                "serial_symlink": serial_link,
                "status_rate_hz": status_rate_hz,
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r empty.sdf",
                description="Arguments passed to Gazebo Sim. Use '-r -s empty.sdf' for headless.",
            ),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=f"pendulum_{os.getpid()}",
                description=(
                    "Gazebo Transport partition. The per-launch default prevents "
                    "old Gazebo instances from mixing /clock and joint_state data."
                ),
            ),
            DeclareLaunchArgument(
                "serial_link",
                default_value="/tmp/pendulum_real_serial",
                description="Pseudo-serial symlink used by the existing Python GUI.",
            ),
            DeclareLaunchArgument(
                "status_rate_hz",
                default_value="100.0",
                description="Status packet rate sent to the pseudo serial port.",
            ),
            DeclareLaunchArgument(
                "enable_serial_bridge",
                default_value="true",
                description="Start the pseudo-serial controller bridge.",
            ),
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            LogInfo(msg=["Gazebo Transport partition: ", gz_partition]),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            bridge,
            serial_bridge,
        ]
    )
