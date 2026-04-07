# inclio_ros2 — ROS2 wrapper for IncLIO

This package wraps the IncLIO LiDAR-Inertial Odometry library as a
standard ROS2 composable node.

---

## Directory layout

```
ros2_wrapper/              ← this package (place it in your ROS2 workspace)
├── CMakeLists.txt
├── package.xml
├── include/ros2_wrapper/
│   ├── cloud_convert.hpp  ← PointCloud2 / Livox → FullCloudPtr converter
│   └── lio_node.hpp       ← composable node declaration
├── src/
│   ├── cloud_convert.cpp
│   └── lio_node.cpp
├── launch/
│   ├── inclio_velodyne.launch.py
│   └── inclio_livox.launch.py
└── config/
    ├── velodyne_ros2.yaml
    └── livox_mid360_ros2.yaml

# Your existing IncLIO source (sibling of ros2_wrapper by default)
../                        ← IncLIO root (CMakeLists.txt, src/, include/, …)
```

---

## Layout

```
ros2_ws/src/
└── inclio_ros2/          ← this package (clone / copy here)
    ├── CMakeLists.txt
    ├── package.xml
    ├── inclio/           ← your IncLIO source goes here as a subfolder
    │   ├── CMakeLists.txt
    │   ├── src/
    │   ├── include/
    │   ├── common/
    │   ├── tools/
    │   └── config/
    ├── src/              ← ROS2 wrapper sources
    ├── include/
    ├── launch/
    └── config/
```

## Build

### 1. Set up the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone / copy this package
git clone <your-inclio_ros2-repo> inclio_ros2
# OR just copy the folder:
cp -r /path/to/ros2_wrapper ~/ros2_ws/src/inclio_ros2

# Place your IncLIO source inside it
cp -r /path/to/IncLIO ~/ros2_ws/src/inclio_ros2/inclio
# OR if it is already there, nothing to do
```

### 2. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

That's it. colcon finds `inclio_ros2/package.xml`, runs its
`CMakeLists.txt`, which calls `add_subdirectory(inclio)` to build
`inclio_common` and `inclio` from source, then compiles the ROS2 node
against those targets — all in one shot, no extra steps.

> **`run_bag` and plain CMake still work** — just build IncLIO normally
> from its own folder:
> ```bash
> cd /path/to/IncLIO
> cmake -B build && cmake --build build
> ```
> Nothing in the IncLIO `CMakeLists.txt` needs to change.

### Livox support

If `livox_ros_driver2` is found by CMake, the Livox `CustomMsg` subscriber
is compiled in automatically (the `HAVE_LIVOX_ROS_DRIVER2` define is set).
No manual edits needed.

---

## Running

### Velodyne / generic PointCloud2

```bash
ros2 launch inclio_ros2 inclio_velodyne.launch.py \
    config_file:=$(ros2 pkg prefix inclio_ros2)/share/inclio_ros2/config/velodyne_ros2.yaml \
    imu_topic:=/imu/data \
    lidar_topic:=/velodyne_points
```

### Livox Mid-360

```bash
ros2 launch inclio_ros2 inclio_livox.launch.py \
    config_file:=$(ros2 pkg prefix inclio_ros2)/share/inclio_ros2/config/livox_mid360_ros2.yaml
```

### Ad-hoc (no launch file)

```bash
ros2 run inclio_ros2 inclio_ros2_node \
    --ros-args \
    -p config_file:=/path/to/config.yaml \
    -r inclio/imu:=/imu/data \
    -r inclio/points:=/velodyne_points
```

---

## Subscribed topics

| Topic                  | Type                                    | Notes                          |
|------------------------|-----------------------------------------|--------------------------------|
| `~/imu`                | `sensor_msgs/msg/Imu`                   | IMU measurements               |
| `~/points`             | `sensor_msgs/msg/PointCloud2`           | Any PointCloud2 LiDAR          |
| `~/points_livox`       | `livox_ros_driver2/msg/CustomMsg`       | Livox native (if compiled in)  |

---

## Published topics

| Topic            | Type                              | Notes                               |
|------------------|-----------------------------------|-------------------------------------|
| `~/odometry`     | `nav_msgs/msg/Odometry`           | Pose + velocity in world frame      |
| `~/path`         | `nav_msgs/msg/Path`               | Full trajectory history             |
| `~/cloud_body`   | `sensor_msgs/msg/PointCloud2`     | Current scan in body (IMU) frame    |
| `~/cloud_world`  | `sensor_msgs/msg/PointCloud2`     | Current scan in world frame         |

### TF

Broadcasts `world → body` at each lidar scan (disable with `publish_tf:=false`).

---

## Parameters

| Parameter          | Default    | Description                                     |
|--------------------|------------|-------------------------------------------------|
| `config_file`      | *(empty)*  | **Required.** Path to IncLIO YAML config        |
| `imu_topic`        | `imu`      | Suffix appended to the node namespace           |
| `lidar_topic`      | `points`   | Suffix appended to the node namespace           |
| `lidar_type`       | `2`        | 1=Livox  2=Velodyne/PC2  3=Ouster               |
| `num_scans`        | `16`       | Number of scan rings                            |
| `time_scale`       | `1e-3`     | Per-point time field → seconds                  |
| `point_filter_num` | `1`        | Keep every N-th point                           |
| `world_frame`      | `world`    | Parent TF frame                                 |
| `body_frame`       | `body`     | Child TF frame (IMU frame)                      |
| `publish_tf`       | `true`     | Broadcast TF                                    |
| `publish_path`     | `true`     | Publish trajectory path                         |
| `publish_cloud`    | `true`     | Publish body/world clouds                       |

---

## Visualising in RViz2

```bash
rviz2
```

Add these displays:

- **Odometry** → topic `inclio/odometry`
- **Path** → topic `inclio/path`
- **PointCloud2** → topic `inclio/cloud_world`
- **TF** (set fixed frame to `world`)

---

## Architecture notes

```
sensor_msgs/Imu  ──►  ImuCallback()  ──►  LIO::AddIMU()
                                                 │
sensor_msgs/PointCloud2                          │ MessageSync
   or CustomMsg   ──►  CloudConverter  ──►  LIO::AddCloud()
                             │                   │
                      FullCloudPtr         ProcessMeasurements()
                                                 │
                                    ┌────────────┼──────────────┐
                               Odometry       Path           TF
                              (~/odometry)  (~/path)    (world→body)
```

The converter (`CloudConverter`) reuses exactly the same field-lookup
logic as `run_bag.cc::parse_pointcloud2()` and `parse_livox_custom()`,
ensuring identical behaviour for all supported LiDAR models.

`lio_mutex_` serialises `AddIMU` and `AddCloud` calls so the node is safe
to use with multi-threaded ROS2 executors.
