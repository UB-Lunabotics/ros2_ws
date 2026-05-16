import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# ---------------------------------------------------------------------------
# Edit these — everything else is derived automatically
# ---------------------------------------------------------------------------
PKG          = "robot_urdf"
URDF         = "robot.urdf.xacro"      # relative to <pkg>/urdf/
WORLD        = "ucf.world"             # relative to <pkg>/worlds/
SPAWN        = (0.0, 0.0, 0.1779, 0.0)  # x, y, z, yaw

# Gazebo camera bridge topics — must match your SDF sensor <topic> names
RGB_TOPIC    = "/camera/color/image"
DEPTH_TOPIC  = "/camera/color/depth_image"
INFO_TOPIC   = "/camera/color/camera_info"

# Odometry topics
WHEEL_ODOM   = "/odom"               # diff drive plugin output
ODOM_TOPIC   = "/odometry/filtered"  # EKF output — consumed by RTAB-Map/Nav2

SCAN_TOPIC   = "/scan"
# ---------------------------------------------------------------------------


def generate_launch_description():
    pkg = get_package_share_directory(PKG)

    localization = LaunchConfiguration("localization")
    rviz         = LaunchConfiguration("rviz")

    # -------------------------------------------------------------------------
    # Robot description
    # -------------------------------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command([
                    FindExecutable(name="xacro"), " ",
                    os.path.join(pkg, "urdf", URDF),
                ]),
                value_type=str  # ← forces string, bypasses YAML parser
            ),
            "use_sim_time": True,
            "publish_frequency": 50.0,
        }],
    )

    # -------------------------------------------------------------------------
    # Gazebo Fortress
    # -------------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
        )),
        launch_arguments={
            "gz_args": f"-r -v 4 {os.path.join(pkg, 'worlds', WORLD)}"
        }.items(),
    )

    # for simulating the localization the zed2i provides
    zed_noise = Node(
        package='robot_urdf',
        executable='zed_noise_odom.py',
        name='zed_noise_odom',
        parameters=[{
            'use_sim_time': True,
            'pos_noise_std':   0.01,
            'angle_noise_std': 0.005,
            'robot_name':      'robot',   # must match your Gazebo model name
        }]
    )

    # Spawn after a delay to give Gazebo time to fully load the world.
    # The 'create' node retries internally, but it needs the server to be
    # accepting requests — 10s is conservative but reliable.
    spawn = TimerAction(
        period=10.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "robot", "-topic", "robot_description",
                "-x", str(SPAWN[0]), "-y", str(SPAWN[1]),
                "-z", str(SPAWN[2]), "-Y", str(SPAWN[3]),
            ],
            output="screen",
        )],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"use_sim_time": True},
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "qos_overrides./tf_static.publisher.reliability": "reliable",
            },
        ],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            #f"{WHEEL_ODOM}@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            #"/visual_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            f"{RGB_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{DEPTH_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/color/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            f"{INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{SCAN_TOPIC}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/world/arena/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        ],
    )

    # -------------------------------------------------------------------------
    # EKF — fuses wheel odom + Gazebo IMU
    # Starts after spawn delay so /odom is already being published
    # -------------------------------------------------------------------------
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg, "config", "ekf.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[("odometry/filtered", ODOM_TOPIC)],
    )

    # -------------------------------------------------------------------------
    # RTAB-Map
    # -------------------------------------------------------------------------
    rtab_cfg = os.path.join(pkg, "config", "rtabmap_params.yaml")
    _common = {
        "use_sim_time": True, 
        "subscribe_depth": True, 
        "subscribe_rgb": True,
        "frame_id": "base_link", 
        "odom_frame_id": "odom", 
        "approx_sync": True,
    }
    _remaps = [
        ("rgb/image",       RGB_TOPIC),
        ("depth/image",     DEPTH_TOPIC),
        ("rgb/camera_info", INFO_TOPIC),
        ("odom",            ODOM_TOPIC),   # /odometry/filtered from EKF\
        ("scan_cloud",      "/camera/color/points"),
    ]

    rtab_slam = Node(
        package="rtabmap_slam", executable="rtabmap",
        parameters=[rtab_cfg, {
            **_common,
            "map_frame_id": "map",
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
            "database_path": "~/.ros/rtabmap.db",
            "Mem/IncrementalMemory": "true", 
            "Mem/InitWMWithAllNodes": "false",
        }],
        remappings=[*_remaps, ("scan", SCAN_TOPIC)],
        arguments=["--delete_db_on_start"],
        condition=UnlessCondition(localization),
        output="screen",
    )

    rtab_loc = Node(
        package="rtabmap_slam", executable="rtabmap",
        parameters=[rtab_cfg, {
            **_common,
            "map_frame_id": "map", "publish_tf": True,
            "Mem/IncrementalMemory": "false", "Mem/InitWMWithAllNodes": "true",
        }],
        remappings=_remaps,
        condition=IfCondition(localization),
        output="screen",
    )

    # -------------------------------------------------------------------------
    # Nav2
    # -------------------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"
        )),
        launch_arguments={
            "use_sim_time": "true",
            "params_file":  os.path.join(pkg, "config", "nav2_params.yaml"),
            "autostart":    "true",
        }.items(),
    )

    # -------------------------------------------------------------------------
    # RViz2
    # -------------------------------------------------------------------------
    rviz_cfg = os.path.join(pkg, "rviz", "robot.rviz")
    rviz_node = Node(
        package="rviz2", executable="rviz2",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_cfg] if os.path.exists(rviz_cfg) else [],
        parameters=[{"use_sim_time": True}],
    )

    # -------------------------------------------------------------------------
    # Startup sequence
    #  t=0s   — RSP, Gazebo, bridge, RViz start immediately
    #  t=10s  — spawn robot (Gazebo world should be loaded by now)
    #  t=13s  — EKF + RTAB-Map (odom should be flowing from diff drive plugin)
    #  t=16s  — Nav2 (map + odom TF should be established)
    # -------------------------------------------------------------------------
    return LaunchDescription([
        DeclareLaunchArgument("localization", default_value="false",
                              description="true = localization only (needs existing map DB)"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz2"),

        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", os.path.join(pkg, "models")),

        rsp, gazebo, bridge, rviz_node,

        spawn,  # delayed 10s
        TimerAction(period=13.0, actions=[zed_noise])
        #TimerAction(period=13.0, actions=[ekf, rtab_slam, rtab_loc]),
        #TimerAction(period=16.0, actions=[nav2]),
    ])