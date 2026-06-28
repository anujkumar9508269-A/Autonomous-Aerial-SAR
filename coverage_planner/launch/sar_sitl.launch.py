"""
launch/sar_sitl.launch.py
==========================
Full SAR pipeline launch for IIT Indore SITL simulation.

Starts:
  1. Gazebo Classic with iiti_sar.world
  2. ArduPilot SITL (spawned as a subprocess via ExecuteProcess)
  3. MAVROS (connects SITL ↔ ROS 2)
  4. coverage_planner_node  (publishes /coverage_path)
  5. waypoint_follower       (follows path via MAVROS)

Usage:
  # Terminal 1 — full pipeline (SAR test zone, nonconvex strategy, 30 m alt)
  ros2 launch coverage_planner sar_sitl.launch.py

  # Override polygon / strategy / altitude at launch time:
  ros2 launch coverage_planner sar_sitl.launch.py \\
      polygon_file:=/path/to/iiti_campus.yaml \\
      strategy:=lawnmower \\
      altitude:=25.0

Prerequisites (run once before first launch):
  # 1. ArduPilot SITL built
  cd ~/ardupilot && ./waf copter          # build once

  # 2. GeographicLib datasets
  sudo $(python3 -c "import mavros; print(mavros.__path__[0])")/scripts/install_geographiclib_datasets.sh

  # 3. Python deps
  pip3 install shapely>=2.0 numpy pyyaml triangle

  # 4. Workspace built
  cd ~/sar_ws && colcon build --packages-select coverage_planner
  source install/setup.bash

Troubleshooting:
  - MAVROS "FCU URL" error  → SITL not started; check ardupilot path below
  - EKF not initialised     → wait ~15 s after Gazebo opens; SITL needs
                               Gazebo physics ticking to converge EKF
  - Drone never arms        → GPS fix type < 3; wait longer or set
                               SIM_GPS_DELAY=0 in SITL console
"""

import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


# ── Helper: find ArduPilot SITL binary ─────────────────────────────────────
def _ardupilot_bin() -> str:
    """Return path to ArduCopter SITL binary. Edit if your clone lives elsewhere."""
    candidates = [
        os.path.expanduser('~/ardupilot/build/sitl/bin/arducopter'),
        os.path.expanduser('~/ardupilot/ArduCopter/ArduCopter.elf'),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    # fallback — will fail with a clear error at runtime
    return os.path.expanduser('~/ardupilot/build/sitl/bin/arducopter')


def generate_launch_description():

    pkg_share = get_package_share_directory('coverage_planner')

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_polygon_file = DeclareLaunchArgument(
        'polygon_file',
        default_value=os.path.join(pkg_share, 'config', 'iiti_sar_zone.yaml'),
        description='Path to mission polygon YAML (gps_polygon or polygon key)',
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

    # ── Environment variables ─────────────────────────────────────────────
    # Tell Gazebo where to find our world file and models
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg_share, 'models') + ':' +
        os.environ.get('GAZEBO_MODEL_PATH', ''),
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

    # ── 2. ArduPilot SITL ─────────────────────────────────────────────────
    # Spawned after Gazebo (TimerAction below) so Gazebo physics is running.
    # SITL home = IIT Indore main gate area (22.5200°N, 75.9212°E)
    #
    # --model=gazebo      → use Gazebo plugin for physics (needs ardupilot-gazebo plugin)
    # --model=quad        → pure SITL, no Gazebo model plugin (simpler; use this first)
    #
    # NOTE: If you have ardupilot-gazebo plugin installed, change --model=quad
    #       to --model=gazebo to get proper Gazebo-coupled flight physics.
    sitl = ExecuteProcess(
        cmd=[
            _ardupilot_bin(),
            '--model=quad',
            '--speedup=1',
            '--home=22.5200,75.9212,535,0',   # lat,lon,alt_AMSL(m),heading
            '--defaults=' + os.path.expanduser(
                '~/ardupilot/Tools/autotest/default_params/copter.parm'
            ),
        ],
        output='screen',
        name='ardupilot_sitl',
        # Run from ArduPilot dir so relative paths inside SITL work
        cwd=os.path.expanduser('~/ardupilot'),
    )

    # ── 3. MAVROS ─────────────────────────────────────────────────────────
    # Connects to SITL on UDP 14551 (default ArduPilot port)
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
            # Tell MAVROS to publish poses on /mavros/local_position/pose
            # with RELIABLE QoS so waypoint_follower sees them.
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
    follower_node = Node(
        package='coverage_planner',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'acceptance_radius': 3.0,    # metres; increase if SITL GPS is noisy
            'takeoff_wait_s':    8.0,
            'guided_timeout_s': 15.0,
            'cte_gain':          0.4,
            'wp_timeout_s':     45.0,    # longer for slow SITL
            'min_gps_fix_type':  3,
            'min_battery_pct':   0.0,    # disable battery check in SITL
        }],
    )

    # ── Sequencing ────────────────────────────────────────────────────────
    # Gazebo → SITL (after 5 s) → MAVROS (after 10 s) → nodes (after 15 s)
    sitl_delayed = TimerAction(period=5.0, actions=[sitl])
    mavros_delayed = TimerAction(period=10.0, actions=[mavros_node])
    nodes_delayed = TimerAction(period=15.0, actions=[planner_node, follower_node])

    return LaunchDescription([
        # Declare args first
        declare_polygon_file,
        declare_strategy,
        declare_altitude,
        declare_hfov,
        declare_vfov,
        declare_overlap,
        # Env
        set_gazebo_model_path,
        # Processes
        gazebo,
        sitl_delayed,
        mavros_delayed,
        nodes_delayed,
    ])
