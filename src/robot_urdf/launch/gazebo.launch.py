import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # Load the URDF so robot_state_publisher can publish TFs.
    pkg_share = get_package_share_directory("robot_urdf")
    urdf_path = os.path.join(pkg_share, "urdf", "Assem1.urdf")
    controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    # Ensure Gazebo can resolve package:// meshes.
    resource_path = os.path.join(pkg_share)
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{resource_path}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}",
    )

    # Gazebo Harmonic uses ros_gz_sim.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True},
        ],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "Assem1", "-world", "default"],
        output="screen",
    )

    # Spawn controllers after the model is in Gazebo.
    # gz_ros2_control exposes controller_manager under /model/<name>/controller_manager.
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/model/Assem1/controller_manager",
            "--ros-args",
            "--params-file",
            controllers_path,
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/model/Assem1/controller_manager",
            "--ros-args",
            "--params-file",
            controllers_path,
        ],
        output="screen",
    )

    drum_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "drum_controller",
            "--controller-manager",
            "/model/Assem1/controller_manager",
            "--ros-args",
            "--params-file",
            controllers_path,
        ],
        output="screen",
    )

    controller_spawners = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_spawner, diff_drive_spawner, drum_spawner],
        )
    )

    return LaunchDescription(
        [
            set_resource_path,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            controller_spawners,
        ]
    )
