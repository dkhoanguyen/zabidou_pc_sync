# ROS 2 Workspace

This workspace contains a minimal ROS 2 package that publishes a colored Helios
point cloud for viewing in `rviz2`.

Package:

- `lucid_colored_cloud`

Node:

- `colored_cloud_publisher`

Expected prerequisites:

- a working ROS 2 installation sourced into the shell
- the `calibration` project already built at `calibration/build`
- connected Phoenix and Helios cameras

Build:

```bash
source /opt/ros/<distro>/setup.bash
cmake -S calibration -B calibration/build
cmake --build calibration/build -j4
cd ros2_ws
colcon build
source install/setup.bash
```

Run:

```bash
ros2 run lucid_colored_cloud colored_cloud_publisher
```

Useful overrides:

```bash
ros2 run lucid_colored_cloud colored_cloud_publisher --ros-args \
  -p stereo_yaml:=/home/khoa/Projects/zabidou_pc_sync/calibration/results/combined_all_datasets/stereo_calibration.yaml \
  -p topic:=/helios/colored_points \
  -p frame_id:=helios_frame \
  -p publish_period_ms:=300
```

To view in `rviz2`:

1. Start `rviz2`
2. Add a `PointCloud2` display
3. Set topic to `/helios/colored_points`
4. Set Fixed Frame to `helios_frame`
