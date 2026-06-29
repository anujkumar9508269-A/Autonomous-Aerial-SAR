"""
launch/sar_sitl.launch.py
==========================
Full SAR pipeline launch — INTEGRATED (coverage + detection + geo-tagging).

Nodes launched:
  1. Gazebo Classic  — iiti_sar.world (with spawned drone+camera)
  2. ArduPilot SITL  — coupled to Gazebo via ardupilot_gazebo plugin
  3. MAVROS          — SITL ↔ ROS 2 bridge
  4. coverage_planner_node  → publishes /coverage_path
  5. waypoint_follower      → follows path, arms/takes off/lands
  6. human_detection_node   → YOLO on /camera/image_raw → /detections
  7. geotagger              → /detections + pose → /sar/raw_tags
  8. position_estimator     → /sar/raw_tags → /sar/confirmed_tags
  9. results_logger         → /sar/confirmed_tags → CSV + GeoJSON

Usage:
  ros2 launch coverage_planner sar_sitl.launch.py

  # Override polygon / strategy / altitude:
  ros2 launch coverage_planner sar_sitl.launch.py \\
      polygon_file:=/path/to/iiti_campus.yaml \\
      strategy:=lawnmower \\
      altitude:=25.0

Prerequisites (run once):
  # 1. Build ArduPilot SITL
  cd ~/ardupilot && ./waf copter

  # 2. Install ardupilot_gazebo plugin (provides Gazebo-coupled drone model)
  # See: https://github.com/ArduPilot/ardupilot_gazebo
  # After install, GAZEBO_MODEL_PATH must include its models/ directory.

  # 3. GeographicLib datasets
  sudo $(python3 -c "import mavros; print(mavros.__path__[0])")/scripts/install_geographiclib_datasets.sh

  # 4. Python deps
  pip3 install shapely>=2.0 numpy pyyaml triangle ultralytics

  # 5. Build all packages
  cd ~/sar_ws && colcon build --packages-select coverage_planner human_detection sar_drone
  source install/setup.bash

  # 6. Set ARMING_SKIPCHK=1 in SITL so waypoint_follower can arm without mavproxy:
  # Start SITL once manually, connect mavproxy, run:
  #   param set ARMING_SKIPCHK 1
  #   param save
  # Then Ctrl+C and use this launch file from now on.

Sequencing:
  t=0s   Gazebo starts (world + drone model spawned)
  t=5s   ArduPilot SITL starts (needs Gazebo physics ticking)
  t=12s  MAVROS starts (needs SITL running)
  t=18s  All ROS nodes start (planner, follower, detection, geo-tagging)

Troubleshooting:
  - No camera feed      → ardupilot_gazebo plugin not installed, or
                          GAZEBO_MODEL_PATH does not include its models/
  - Drone never arms    → Run: param set ARMING_SKIPCHK 1 in SITL console
  - EKF not init        → Wait ~20s after Gazebo opens; normal for SITL
  - geotagger no data   → Check /mavros/local_position/pose is publishing
                          (ros2 topic hz /mavros/local_position/pose)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Helper: locate ArduCopter SITL binary
# ---------------------------------------------------------------------------

def _ardupilot_bin() -> str:
    candidates = [
        os.path.expanduser('~/ardupilot/build/sitl/bin/arducopter'),
        os.path.expanduser('~/ardupilot/ArduCopter/ArduCopter.elf'),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return os.path.expanduser('~/ardupilot/build/sitl/bin/arducopter')


# ---------------------------------------------------------------------------
# Helper: locate ardupilot_gazebo models (for camera-equipped drone model)
# ---------------------------------------------------------------------------

def _ardupilot_gazebo_models() -> str:
    """
    Returns the models path for the ardupilot_gazebo plugin.
    This provides the 'iris_with_ardupilot_camera' model that publishes
    /camera/image_raw and /camera/camera_info into Gazebo/ROS 2.

    Install ardupilot_gazebo from:
      https://github.com/ArduPilot/ardupilot_gazebo
    Typical install path after 'sudo make install':
      /usr/share/ardupilot_gazebo/models
    """
    candidates = [
        '/usr/share/ardupilot_gazebo/models',
        os.path.expanduser('~/ardupilot_gazebo/models'),
        os.path.expanduser('~/ardupilot_gazebo/install/share/ardupilot_gazebo/models'),
    ]
    for c in candidates:
        if os.path.isdir(c):
            return c
    # Fallback — will show a Gazebo "model not found" error at runtime
    # if ardupilot_gazebo is not installed. See Prerequisites above.
    return '/usr/share/ardupilot_gazebo/models'


def generate_launch_description():

    pkg_share = get_package_share_directory('coverage_planner')

    # ── Detect YOLO model path ────────────────────────────────────────────
    # FIX-5: detection_node had an absolute hardcoded path to Anuj's machine.
    # We resolve it relative to the installed human_detection package so it
    # works on any machine after 'colcon build'.
    try:
        human_det_share = get_package_share_directory('human_detection')
    except Exception:
        human_det_share = os.path.expanduser('~/sar_ws/src/human_detection')

    # Primary: look for best.onnx in the package share (after colcon build,
    # data_files would copy it there if listed in setup.py).
    # Secondary: fall back to the source tree path used by Anuj's machine.
    onnx_candidates = [
        os.path.join(human_det_share, 'models', 'best.onnx'),
        os.path.join(human_det_share, 'best.onnx'),
        os.path.expanduser(
            '~/sar_ws/src/human_detection/human_detection/best.onnx'
        ),
        os.path.expanduser(
            '~/drone_ws/src/human_detection/human_detection/best.onnx'
        ),
    ]
    yolo_model_path = next(
        (p for p in onnx_candidates if os.path.isfile(p)),
        onnx_candidates[0],   # fallback — detection_node will log the error
    )

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_polygon_file = DeclareLaunchArgument(
        'polygon_file',
        default_value=os.path.join(pkg_share, 'config', 'iiti_sar_zone.yaml'),
        description='Path to mission polygon YAML',
    )
    declare_strategy = DeclareLaunchArgument(
        'strategy',
        default_value='nonconvex',
        description='Coverage strategy: lawnmower | nonconvex | spiral',
    )
    declare_altitude = DeclareLaunchArgument(
        'altitude',
        default_value='30.0',
        description='Mission altitude in metres AGL',
    )
    declare_hfov = DeclareLaunchArgument(
        'hfov_deg', default_value='60.0',
        description='Camera horizontal FOV in degrees',
    )
    declare_vfov = DeclareLaunchArgument(
        'vfov_deg', default_value='60.0',
        description='Camera vertical FOV in degrees',
    )
    declare_overlap = DeclareLaunchArgument(
        'overlap', default_value='0.1',
        description='Strip overlap fraction (0.1 = 10%)',
    )

    # ── Environment: tell Gazebo where to find drone + world models ───────
    # FIX-1 (camera): ardupilot_gazebo provides the iris_with_ardupilot_camera
    # model which spawns a drone with a downward-facing camera, publishing
    # /camera/image_raw and /camera/camera_info into ROS 2.
    # Without this, there is NO camera feed and detection/geo-tagging get nothing.
    ardupilot_gz_models = _ardupilot_gazebo_models()
    pkg_models = os.path.join(pkg_share, 'models')

    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        ardupilot_gz_models + ':' +
        pkg_models + ':' +
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        ardupilot_gz_models + ':' +
        os.environ.get('GAZEBO_RESOURCE_PATH', ''),
    )

    # ── 1. Gazebo Classic ─────────────────────────────────────────────────
    world_file = os.path.join(pkg_share, 'worlds', 'iiti_sar.world')
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file,
        ],
        output='screen',
        name='gazebo',
    )

    # ── 2. ArduPilot SITL (Gazebo-coupled) ───────────────────────────────
    # FIX-1 (camera): Changed --model=quad  →  --model=gazebo
    # --model=gazebo couples SITL to the Gazebo physics + the
    # ardupilot_gazebo plugin's drone model (iris_with_ardupilot_camera),
    # which has a downward camera publishing /camera/image_raw.
    # --model=quad is pure SITL with no Gazebo visual model and NO camera.
    sitl = ExecuteProcess(
        cmd=[
            _ardupilot_bin(),
            '--model=gazebo',          # FIX: was --model=quad (no camera)
            '--speedup=1',
            '--home=22.5200,75.9212,535,0',
            '--defaults=' + os.path.expanduser(
                '~/ardupilot/Tools/autotest/default_params/copter.parm'
            ),
        ],
        output='screen',
        name='ardupilot_sitl',
        cwd=os.path.expanduser('~/ardupilot'),
    )

    # ── 3. MAVROS ─────────────────────────────────────────────────────────
    mavros_config = os.path.join(pkg_share, 'config', 'mavros_params.yaml')
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='mavros',
        output='screen',
        parameters=[
            {'fcu_url': 'udp://127.0.0.1:14551@14555'},
            {'gcs_url': ''},
            {'target_system_id': 1},
            {'target_component_id': 1},
            {'local_position/frame_id': 'map'},
            mavros_config if os.path.isfile(mavros_config) else {},
        ],
    )

    # ── 4. Coverage planner node ──────────────────────────────────────────
    planner_node = Node(
        package='coverage_planner',
        executable='coverage_planner_node',
        name='coverage_planner_node',
        output='screen',
        parameters=[{
            'polygon_file': LaunchConfiguration('polygon_file'),
            'strategy':     LaunchConfiguration('strategy'),
            'altitude':     LaunchConfiguration('altitude'),
            'hfov_deg':     LaunchConfiguration('hfov_deg'),
            'vfov_deg':     LaunchConfiguration('vfov_deg'),
            'overlap':      LaunchConfiguration('overlap'),
        }],
    )

    # ── 5. Waypoint follower ──────────────────────────────────────────────
    # This is the ONLY node that arms/takes off/lands.
    # Do NOT launch sar_drone/flight_controller — it conflicts with this.
    # FIX-3: flight_controller removed from integrated launch.
    follower_node = Node(
        package='coverage_planner',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'acceptance_radius': 3.0,
            'takeoff_wait_s':    8.0,
            'guided_timeout_s': 30.0,
            'cte_gain':          0.4,
            'wp_timeout_s':     45.0,
            'min_gps_fix_type':  0,    # 0 = any fix (SITL)
            'min_battery_pct':   0.0,  # disabled in SITL
            'arm_retry_cooldown': 5.0,
        }],
    )

    # ── 6. Human detection node ───────────────────────────────────────────
    # FIX-5: YOLO model path resolved dynamically above (not hardcoded).
    # Subscribes to /camera/image_raw (published by ardupilot_gazebo drone).
    # Publishes /detections (Detection2DArray) consumed by geotagger.
    detection_node = Node(
        package='human_detection',
        executable='detection_node',
        name='human_detection_node',
        output='screen',
        parameters=[{
            'model_path': yolo_model_path,
        }],
        # Remap if your ardupilot_gazebo camera publishes on a different topic.
        # Default ardupilot_gazebo topic is /camera/image_raw — matches detection_node.
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
        ],
    )

    # ── 7. Geo-tagger ─────────────────────────────────────────────────────
    # FIX-2: QoS mismatch fixed in geotagger.py (BEST_EFFORT for MAVROS topics).
    # FIX-4: camera_info topic remapped to match ardupilot_gazebo output.
    # FIX-7: z_ground exposed as parameter (0.0 = ground at Gazebo world origin).
    geotagger_node = Node(
        package='sar_drone',
        executable='geotagger',
        name='geotagger',
        output='screen',
        parameters=[{
            'z_ground': 0.0,    # ground altitude in map frame; 0.0 for flat world
        }],
        remappings=[
            # FIX-4: geotagger subscribed to /drone/camera/camera_info
            # ardupilot_gazebo publishes /camera/camera_info — remap here.
            ('/drone/camera/camera_info', '/camera/camera_info'),
        ],
    )

    # ── 8. Position estimator ─────────────────────────────────────────────
    # Running mean filter over raw geo-tags per track_id.
    # No fixes needed — topic names match correctly.
    position_estimator_node = Node(
        package='sar_drone',
        executable='position_estimator',
        name='position_estimator',
        output='screen',
    )

    # ── 9. Results logger ─────────────────────────────────────────────────
    # Writes CSV + GeoJSON to ~/sar_ws/results/ on shutdown.
    # NOTE (FIX-6): coordinates saved are local ENU (metres), not GPS.
    # For proper GeoJSON (lon/lat), geotagger or results_logger should
    # call local_to_gps() from coverage_planner.planner before writing.
    # For the demo this is acceptable — positions are still accurate in
    # the map frame and can be post-processed.
    results_logger_node = Node(
        package='sar_drone',
        executable='results_logger',
        name='results_logger',
        output='screen',
    )

    # ── Sequencing ────────────────────────────────────────────────────────
    # t=0s   Gazebo starts (world loads, physics ticks)
    # t=5s   SITL starts (needs Gazebo running for --model=gazebo)
    # t=12s  MAVROS starts (needs SITL running on UDP 14551)
    # t=18s  All ROS nodes start:
    #           - planner publishes /coverage_path immediately
    #           - waypoint_follower waits for MAVROS connection (up to 30s)
    #           - detection node waits for camera frames
    #           - geotagger waits for pose + camera_info + detections
    #           - position_estimator and results_logger wait for upstream data
    sitl_delayed   = TimerAction(period=5.0,  actions=[sitl])
    mavros_delayed = TimerAction(period=12.0, actions=[mavros_node])
    nodes_delayed  = TimerAction(
        period=18.0,
        actions=[
            planner_node,
            follower_node,
            detection_node,
            geotagger_node,
            position_estimator_node,
            results_logger_node,
        ]
    )

    return LaunchDescription([
        # Declare args
        declare_polygon_file,
        declare_strategy,
        declare_altitude,
        declare_hfov,
        declare_vfov,
        declare_overlap,
        # Environment
        set_gazebo_model_path,
        set_gazebo_resource_path,
        # Processes in order
        gazebo,
        sitl_delayed,
        mavros_delayed,
        nodes_delayed,
    ])