"""
mapping.launch.py
UB Lunabotics 2026 — Manual Mapping Mode

Use this BEFORE competition to build and save a map of the arena.
Drive the robot manually with an Xbox controller while RTAB-Map
builds the map in real time. Save the map when done.

HOW TO USE:
  1. Launch this file:
       ros2 launch lunabot_bringup mapping.launch.py

  2. Drive the robot around the entire arena with the Xbox controller:
       Left stick  → forward/backward
       Right stick → turn left/right
       Hold LB     → enable movement (deadman switch)

  3. When you've covered the whole arena, save the map:
       ros2 service call /rtabmap/trigger_new_map std_srvs/srv/Empty

  4. The map database is saved automatically to:
       ~/.ros/rtabmap.db

  5. For the competition run, load that saved map by setting:
       Mem/IncrementalMemory: "false"  in rtabmap_params.yaml
       This switches RTAB-Map from mapping mode to localization only mode.

XBOX CONTROLLER BUTTON MAP:
  Left stick Y  → linear velocity (forward/back)
  Right stick X → angular velocity (turn)
  LB (button 4) → deadman switch (must hold to move)

⚠️  TODO: confirm joy_dev port — run 'ls /dev/input/js*' to find controller
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# ZED wrapper — only available on Jetson with ZED SDK installed
try:
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_available = True
except Exception:
    zed_wrapper_dir = None
    zed_available = False


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (true for Gazebo, false for real robot)'
    )

    # ── Config File Paths ─────────────────────────────────────────────────────
    bringup_dir    = get_package_share_directory('lunabot_bringup')
    rtabmap_dir    = get_package_share_directory('rtabmap_ros')

    rtabmap_params_file = os.path.join(bringup_dir, 'config', 'rtabmap_params.yaml')

    # =========================================================================
    # LAYER 1: SENSORS  (t = 0s)
    # =========================================================================

    # ZED2i Front Camera
    zed_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name':  'zed_front',
            'use_sim_time': use_sim_time,
        }.items()
    ) if zed_available else LogInfo(msg='[MAPPING] ZED wrapper not found, skipping cameras...')

    # ZED2i Rear Camera
    zed_rear = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name':  'zed_rear',
            'use_sim_time': use_sim_time,
        }.items()
    ) if zed_available else LogInfo(msg='[MAPPING] ZED rear camera skipped...')

    # =========================================================================
    # LAYER 2: XBOX CONTROLLER TELEOP  (t = 0s)
    # Allows manual driving during mapping session
    # =========================================================================

    # Joy node — reads raw input from Xbox controller
    # TODO: confirm joy_dev — run 'ls /dev/input/js*' on Jetson
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',   # TODO: confirm this is your Xbox controller
            'deadzone': 0.1,           # ignore small stick movements
            'autorepeat_rate': 20.0,
        }]
    )

    # Teleop node — converts Xbox input to /cmd_vel velocity commands
    # Left stick Y  = linear.x (forward/back)
    # Right stick X = angular.z (turn)
    # LB held       = deadman switch (must hold or robot won't move)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[{
            'axis_linear.x': 1,        # left stick Y axis
            'axis_angular.yaw': 3,     # right stick X axis
            'scale_linear.x': 0.3,     # max forward speed (m/s) — keep slow for mapping
            'scale_angular.yaw': 0.8,  # max turn speed (rad/s)
            'enable_button': 4,        # LB button — deadman switch
            'require_enable_button': True,
        }]
    )

    # ESP32 Bridge — forwards /cmd_vel to motors so robot actually moves
    esp32_bridge = Node(
        package='rover_autonomy',
        executable='jetson_serial_sender',
        name='jetson_serial_sender',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # =========================================================================
    # LAYER 3: RTAB-MAP  (t = 3s)
    # Builds the map while you drive
    # Mem/IncrementalMemory: true = mapping mode (builds new map)
    # =========================================================================

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rtabmap_args':         '--delete_db_on_start',  # fresh map each session
            'use_sim_time':         use_sim_time,
            'rgb_topic':            '/zed/zed_node/rgb/color/rect/image',
            'depth_topic':          '/zed/zed_node/depth/depth_registered',
            'camera_info_topic':    '/zed/zed_node/rgb/color/rect/camera_info',
            'frame_id':             'base_link',
            'odom_topic':           '/zed/zed_node/odom',
            'params_file':          rtabmap_params_file,
            'publish_tf':           'true',
            'rviz':                'false',  # set to true to visualize mapping in real time (not recommended, resource intensive)
            'rtabmapviz':          'false',
        }.items()
    )

    mapping_layer = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[MAPPING] Starting RTAB-Map in mapping mode...'),
            rtabmap,
        ]
    )

    # =========================================================================
    # ASSEMBLE
    # =========================================================================

    return LaunchDescription([
        declare_use_sim_time,

        LogInfo(msg='[MAPPING] Starting UB Lunabotics mapping session...'),
        LogInfo(msg='[MAPPING] Hold LB on Xbox controller to drive.'),
        LogInfo(msg='[MAPPING] Drive the entire arena before saving the map.'),

        # t=0s — sensors + teleop
        zed_front,
        zed_rear,
        joy_node,
        teleop_node,
        esp32_bridge,

        # t=3s — RTAB-Map starts after cameras are up
        mapping_layer,

        LogInfo(msg='[MAPPING] Ready! Drive the robot to build the map.'),
    ])
