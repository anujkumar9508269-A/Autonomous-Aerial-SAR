# Autonomous Aerial Search and Rescue (PS-3)

**IITISoC 2026 — Team IVAR 1**

An autonomous UAV system for Search and Rescue (SAR) missions. The drone plans a coverage path over a polygonal search zone, detects humans using a YOLOv8 model, and geo-tags each detected person's real-world position — all without manual intervention.

**Team:**
| Member | Module |
|---|---|
| Abhay Rajawat | Coverage Path Planning |
| Anuj Kumar | Human Detection (YOLOv8) |
| Harmehar | Geo-tagging + Position Estimation |

Integration of all modules into the full pipeline is a joint effort of the entire team.

---

## Demo

We will share the drive link soon.

---

## Folder Structure

```
Autonomous-Aerial-SAR/
├── src/
│   ├── coverage_planner/
│   │   ├── coverage_planner/
│   │   │   ├── __init__.py
│   │   │   ├── planner.py              # Lawnmower, non-convex, spiral strategies
│   │   │   └── coverage_planner_node.py    # ROS 2 node → publishes /coverage_path
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── flight_control/
│   │   ├── flight_control/
│   │   │   ├── __init__.py
│   │   │   └── waypoint_follower.py    # ROS 2 node → MAVROS state machine
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── human_detection/
│   │   ├── human_detection/
│   │   │   ├── __init__.py
│   │   │   └── detection_node.py       # ROS 2 node → /detections
│   │   ├── models/
│   │   │   ├── best.onnx               # Optimised ONNX model for inference
│   │   │   └── best.pt                 # PyTorch weights
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── sar_drone/
│   │   ├── sar_drone/
│   │   │   ├── __init__.py
│   │   │   ├── geotagger.py            # Ray-cast projection → /sar/raw_tags
│   │   │   ├── position_estimator.py   # Running mean filter → /sar/confirmed_tags
│   │   │   └── results_logger.py       # Writes CSV + GeoJSON on shutdown
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── launch/
│   │   └── sar_sitl.launch.py          # Master launch file — full pipeline
│   │
│   ├── config/
│   │   ├── iiti_sar_zone.yaml          # SAR test zone polygon (GPS coordinates)
│   │   ├── iiti_campus.yaml            # Full campus polygon
│   │   └── mavros_params.yaml          # MAVROS configuration
│   │
│   ├── docs/                           # Reports and architecture diagrams
│   │
│   ├── results/                        # Auto-generated on mission completion
│   │   ├── geotags_YYYYMMDD_HHMMSS.csv
│   │   └── geotags_YYYYMMDD_HHMMSS.geojson
│   │
│   └── worlds/
│       └── iiti_sar.world              # Gazebo world with 5 human models
```

---

## System Architecture

```
Gazebo + ArduPilot SITL
        │
      MAVROS
        │
        ├──→ coverage_planner_node  →  /coverage_path
        │                                    │
        ├──→ waypoint_follower  ←────────────┘
        │    (arms, takes off, follows path, lands)
        │
        ├──→ /camera/image_raw  →  detection_node  →  /detections
        │
        ├──→ /camera/camera_info  →  geotagger
        │
        ├──→ /mavros/local_position/pose  →  geotagger  ←───┘
        │                                        │
        │                               /sar/raw_tags
        │                                        │
        │                            position_estimator
        │                                        │
        │                           /sar/confirmed_tags
        │                                        │
        │                             results_logger
        │                           (CSV + GeoJSON output)
```

---

## Dependencies

### System
```bash
# ROS 2 Humble
sudo apt install -y ros-humble-desktop ros-humble-mavros ros-humble-mavros-extras
sudo apt install -y ros-humble-vision-msgs

# GeographicLib datasets (required by MAVROS)
sudo $(python3 -c "import mavros; print(mavros.__path__[0])")/scripts/install_geographiclib_datasets.sh

# Gazebo Classic 11
sudo apt install -y gazebo libgazebo11-dev
```

### ArduPilot SITL
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
./waf configure --board sitl
./waf copter
```

### ardupilot_gazebo plugin
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4 && sudo make install

# Add to ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
echo 'export GAZEBO_MODEL_DATABASE_URI=""' >> ~/.bashrc
source ~/.bashrc
```

### Python
```bash
pip3 install shapely>=2.0 pyyaml triangle ultralytics

# you have to delete numpy installed from ultralytics
pip install "numpy<2.0.0"
```

---

## Setup

```bash
# 1. Clone the repository
cd ~/sar_ws/src
git clone <repository-url> .

# 2. Build all packages
cd ~/sar_ws
colcon build --packages-select coverage_planner flight_control human_detection sar_drone
source install/setup.bash
```

---

## How to Run

### Full Pipeline (Recommended)
```bash
cd ~/sar_ws
source install/setup.bash
ros2 launch coverage_planner sar_sitl.launch.py
```

### Override Parameters
```bash
ros2 launch coverage_planner sar_sitl.launch.py \
    polygon_file:=/path/to/custom.yaml \
    strategy:=lawnmower \
    altitude:=25.0
```

### Run Nodes Individually (for testing)

As we are unable to simulate the complete pipeline through a single launch file, we are running each node individually for simulation.

**1. Gazebo**
```bash
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so ~/drone_ws/src/Autonomous-Aerial-SAR/coverage_planner/worlds/iiti_sar.world
```

**2. ArduPilot**
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

**3. MAVROS**
```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```

**4. Detection Node**
```bash
python3 ~/drone_ws/src/Autonomous-Aerial-SAR/human_detection/human_detection/detection_node.py
```

**5. Visualization Node**
```bash
ros2 run rqt_image_view rqt_image_view
```

**6. Geotagger Node**
```bash
ros2 run sar_drone geotagger
```

**7. Position Estimator Node**
```bash
ros2 run sar_drone position_estimator
```

**8. Results Logger Node**
```bash
ros2 run sar_drone results_logger
```

**9. Waypoint Follower Node**
```bash
ros2 run coverage_planner waypoint_follower
```

**10. Coverage Planner Node**
```bash
ros2 run coverage_planner coverage_planner_node
```

### Verify Results
```bash
ls ~/sar_ws/results/
# geotags_YYYYMMDD_HHMMSS.csv
# geotags_YYYYMMDD_HHMMSS.geojson
```

---

## Coverage Strategies

| Strategy | Waypoints | Path Length | Est. Flight Time | Coverage |
|---|---|---|---|---|
| Lawnmower | 8 | 385 m | 1.3 min | 100% |
| Non-convex | 8 | 366 m | 1.2 min | 100% |
| Spiral | 15 | 755 m | 2.5 min | 100% |

*Results on L-shape demo polygon, alt=20 m, overlap=10%*

---

## Pre-flight Check (run before SITL)
```bash
cd ~/sar_ws/src/coverage_planner
python3 scripts/check_iiti_polygon.py
python3 tests/test_planner.py
```
