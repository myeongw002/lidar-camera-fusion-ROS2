# Validation status — `ring-range-image`

This branch changes the core LiDAR representation from PCL `RangeImageSpherical` to a fixed-size range image built from `PointCloud2.ring` and azimuth. Previous validation results from `main` do **not** apply to this branch.

## Intended geometry

Default VLP-16 configuration:

```text
input_rows = 16
output_rows = 64
horizontal_resolution_deg = 0.2
width = round(360 / 0.2) = 1800
```

Expected published range images:

```text
/range_image_raw          16 × 1800, 32FC1, metres
/range_image_interpolated 64 × 1800, 32FC1, metres
```

Invalid cells are `NaN`.

## Automated checks included in the branch

The C++ interpolation test is intended to check:

- fixed raw and interpolated image dimensions;
- metric range preservation on a synthetic VLP-16 scan;
- exactly 64 reconstructed output rows;
- camera-FOV cropping;
- rejection of interpolation across a configured range discontinuity;
- nearest-return retention when points collide in one range-image cell;
- ground correction;
- invalid parameter rejection.

The DDS smoke test is intended to check:

- required `ring:uint16` input field;
- both nodes starting without sensor data;
- SensorDataQoS discovery;
- `/range_image_raw` as `16 × W`, `32FC1`;
- `/range_image_interpolated` as `64 × W`, `32FC1`;
- `NaN` range images and empty point clouds for fully filtered input;
- non-empty interpolated and colored clouds for a valid synthetic VLP-16 scan;
- camera RGB sampling and overlay modification;
- timestamp/frame propagation.

The startup test is intended to check:

- range-image dimensions and vertical-angle configuration;
- invalid resolution/row/range-gap parameters;
- camera calibration regression checks;
- launch-file parsing and idle startup;
- RViz plugin classes.

## Commands to run on Ubuntu 22.04 / ROS2 Humble

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws

colcon build --symlink-install \
  --packages-select lidar_camera_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

colcon test --packages-select lidar_camera_fusion \
  --event-handlers console_direct+
colcon test-result --verbose
```

Then explicitly inspect topics with a real VLP-16 source:

```bash
ros2 topic echo /velodyne_points --once
ros2 topic info /range_image_raw
ros2 topic info /range_image_interpolated
ros2 topic info /pc_interpoled
```

Verify the incoming cloud contains a `ring` field of type `UINT16`.

## Current status

The code in this branch was written through the GitHub connector from an environment that does not contain a ROS2 Humble build/runtime installation. Therefore the branch has **not yet been independently compiled or executed in this session**.

Do not treat the previous `main` branch's successful build/test numbers as validation of this redesign. Run the commands above before merging to `main`.
