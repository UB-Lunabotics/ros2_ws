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
from launch.conditions import IfCondition, UnlessCondition
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
    bringup_dir     = get_package_share_directory('lunabot_bringup')
    nav2_dir        = get_package_share_directory('nav2_bringup')
    rtabmap_dir     = get_package_share_directory('rtabmap_ros')

    nav2_params_file     = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    rtabmap_params_file  = os.path.join(bringup_dir, 'config', 'rtabmap_params.yaml')
    ekf_params_file      = os.path.join(bringup_dir, 'config', 'ekf_params.yaml')
    ekf_params_sim_file  = os.path.join(bringup_dir, 'config', 'ekf_params_sim.yaml')

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
    ) if zed_available else LogInfo(msg='[BRINGUP] ZED wrapper not found, skipping cameras...')

    # ZED2i Rear Camera
    '''  # rear camera currently unused, can be enabled if needed for better rear localization or obstacle detection
    zed_rear = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name':  'zed_rear',
            'use_sim_time': use_sim_time,
        }.items()
    ) if zed_available else LogInfo(msg='[BRINGUP] ZED rear camera skipped...')
    '''

    # AprilTag detector
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_node',
        name='apriltag_ros',
        remappings=[
            ('image_rect',  '/zed_front/zed_node/rgb/color/rect/image'),
            ('camera_info', '/zed_front/zed_node/rgb/color/rect/camera_info'),
        ],
        parameters=[
            os.path.join(bringup_dir, 'config', 'apriltag_config.yaml')
        ]
    )

    # Pose initializer
    pose_initializer = Node(
        package='rover_autonomy',
        executable='pose_initializer',
        name='pose_initializer',
    )

    # IR Sensor Publisher
    # TODO: confirm serial port with electrical team
    # TODO: must be different port from ESP32 bridge
    ir_sensor = Node(
        package='rover_autonomy',
        executable='ir_sensor_publisher',
        name='ir_sensor_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            # topic name now matches brain: /ir_sensor_value as Float32
        ]
    )

    # ESP32 Serial Bridge
    # TODO: confirm /dev/ttyUSB0 is the ESP32 port with electrical team
    esp32_bridge = Node(
        package='rover_autonomy',
        executable='jetson_serial_sender',
        name='jetson_serial_sender',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # =========================================================================
    # LAYER 2: LOCALIZATION  (t = 3s)
    # =========================================================================

    # EKF — real robot
    ekf_node_real = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[
            ekf_params_file,
            {'use_sim_time': False}
        ],
        remappings=[('odometry/filtered', '/odom')],
        condition=UnlessCondition(use_sim_time)
    )

    # EKF — simulation
    ekf_node_sim = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[
            ekf_params_sim_file,
            {'use_sim_time': True}
        ],
        remappings=[('odometry/filtered', '/odom')],
        condition=IfCondition(use_sim_time)
    )

    # RTAB-Map SLAM
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rtabmap_args':         '--delete_db_on_start',
            'use_sim_time':         use_sim_time,
            'rgb_topic':            '/zed_front/zed_node/rgb/color/rect/image',
            'depth_topic':          '/zed_front/zed_node/depth/depth_registered',
            'camera_info_topic':    '/zed_front/zed_node/rgb/color/rect/camera_info',
            'frame_id':             'base_link',
            'odom_topic':           '/zed_front/zed_node/odom',
            'params_file':          rtabmap_params_file,
            'publish_tf':           'true',
            'rviz':                'false',
            'rtabmapviz':          'false',
        }.items()
    )

    localization_layer = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[BRINGUP] Starting localization layer (EKF + RTAB-Map)...'),
            ekf_node_real,
            ekf_node_sim,
            rtabmap,
        ]
    )

    # =========================================================================
    # LAYER 3: NAVIGATION  (t = 8s)
    # =========================================================================

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file':  nav2_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    navigation_layer = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[BRINGUP] Starting Nav2 navigation stack...'),
            nav2,
        ]
    )

    # =========================================================================
    # LAYER 5: MISSION CONTROL  (t = 12s)
    # TODO: update dig/dump coordinates to real arena values
    # Current placeholders in brain: dig=(2.0, 1.0), dump=(0.0, 0.0)
    # =========================================================================

    mission_layer = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[BRINGUP] Starting mission control (autonomy brain)...'),
            Node(
                package='rover_autonomy',
                executable='brain_node',
                name='lunabotics_brain',
                parameters=[{
                    'use_sim_time': use_sim_time,
                }]
            ),
        ]
    )

    # =========================================================================
    # ASSEMBLE
    # =========================================================================

    return LaunchDescription([
        declare_use_sim_time,

        # t=0s
        LogInfo(msg='[BRINGUP] Starting UB Lunabotics autonomy stack...'),
        zed_front,
        #zed_rear,
        ir_sensor,
        esp32_bridge,
        apriltag_node,
        pose_initializer,

        # t=3s
        localization_layer,

        # t=8s
        navigation_layer,

        # t=12s
        mission_layer,

        LogInfo(msg='[BRINGUP] All layers launched. Robot initializing...'),
    ])