from setuptools import setup
import os
from glob import glob

package_name = 'coverage_planning'

setup(
    name=package_name,
    version='3.1.0',
    packages=[package_name],
    data_files=[
        # ── ROS 2 ament index registration ────────────────────────────
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),

        # ── Launch files ───────────────────────────────────────────────
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),

        # ── Config YAMLs (polygon + MAVROS params) ─────────────────────
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),

        # ── Gazebo world files ─────────────────────────────────────────
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world'),
        ),

        # ── KML files (for external visualisation / Google Earth) ──────
        (
            os.path.join('share', package_name, 'kml'),
            glob('*.kml'),
        ),
    ],
    install_requires=[
        'setuptools',
        'shapely>=2.0',     # vectorised contains_xy; compute_coverage_pct
        'numpy',
        'pyyaml',           # polygon YAML loading
        'triangle',         # constrained Delaunay for non-convex decomp
                            # (falls back to shapely if unavailable)
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Optimised drone coverage planner for SAR — all efficiency levels (v3.1)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_planner_node = coverage_planner.coverage_planner_node:main',
            'waypoint_follower     = coverage_planner.waypoint_follower:main',
        ],
    },
)
