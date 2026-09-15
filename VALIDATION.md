# Local validation — 2026-09-15

Final repository state was validated on Ubuntu 22.04.5 with ROS2 Humble,
PCL 1.12.1, OpenCV 4.5.4, Eigen 3.4.0, and Armadillo 10.8.2. The reference
ROS1 source is EPVelasco/lidar-camera-fusion commit
`621de50a9e0d9c360e4ce9f84b0fc16acbcc5918`.

The current and upstream YAML files both parse to this camera projection row:
`[0.0, 0.0, 1.0, 0.0]`. The automated calibration regression check requires
`camera_matrix[2,2] == +1.0` and verifies that a known forward LiDAR point has
positive projected depth and lands inside the 1280×720 calibration image.

## Build and registered executables

Build/install/log data was kept under `/tmp` so validation did not alter other
packages in the surrounding workspace.

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/lidar-fusion-review/log build \
  --base-paths /home/hrilab/ROS2/compa_ws/src/llidar_camera_fusion \
  --build-base /tmp/lidar-fusion-review/build \
  --install-base /tmp/lidar-fusion-review/install \
  --symlink-install --packages-select lidar_camera_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /tmp/lidar-fusion-review/install/setup.bash
ros2 pkg executables lidar_camera_fusion
```

Result: the Release build succeeded without compiler warnings. ROS2 reported:

```text
lidar_camera_fusion interpolated_node
lidar_camera_fusion lidar_camera_node
```

## Integrated tests

```bash
source /opt/ros/humble/setup.bash
source /tmp/lidar-fusion-review/install/setup.bash
colcon --log-base /tmp/lidar-fusion-review/log test \
  --build-base /tmp/lidar-fusion-review/build \
  --install-base /tmp/lidar-fusion-review/install \
  --packages-select lidar_camera_fusion --event-handlers console_direct+
colcon --log-base /tmp/lidar-fusion-review/log test-result \
  --test-result-base /tmp/lidar-fusion-review/build --verbose
```

Result: all 3 CTest entries passed (`interpolation_test`, `ros_smoke_test`, and
`startup_test`). `colcon test-result` reported 6 tests, 0 errors, 0 failures,
and 0 skipped. Both Python suites are registered with `ament_cmake_pytest` and
use isolated ROS domain IDs and writable build-local ROS log directories.

Coverage exercised in the final run:

- spherical range-image interpolation, densification, finite/range checks,
  FOV, ground correction, filtering, and invalid settings;
- both nodes starting and waiting without sensor data;
- best-effort sensor-data QoS discovery and ApproximateTime fusion;
- empty PointCloud2 output with the LiDAR header for unusable LiDAR frames;
- unchanged fusion image with the camera header for an unusable LiDAR frame;
- non-empty `/pc_interpoled`, `/points2`, and modified `/pcOnImage_image` for
  valid synthetic VLP16-like rings and a BGR camera image;
- finite XYZ output, exact synthetic RGB sampling, and output headers;
- 12 invalid parameter/calibration startup cases, including all array lengths;
- `camera_matrix[2,2] == +1.0`, positive forward projection depth, and a valid
  projected pixel;
- all three launch files loading YAML, starting headlessly, and stopping cleanly;
- RViz YAML parsing and Humble plugin/built-in panel class availability.

## Explicit DDS smoke test

The integrated DDS test passed inside `colcon test`. It was also executed
outside the filesystem/network sandbox to allow normal loopback sockets and to
capture its point count:

```bash
source /tmp/lidar-fusion-review/install/setup.bash
ROS_LOG_DIR=/tmp/lidar-fusion-review/explicit_ros_logs \
ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1 python3 test/ros_smoke_test.py
```

Result: all four outputs passed; `/points2` contained 7,930 colored points.
Headers, RGB values, overlay modification, sensor QoS, empty inputs, nonfinite
inputs, fully filtered inputs, and empty-frame publication all passed.

## Limits

`rosdep` is not installed in this environment, so
`rosdep install --from-paths src --ignore-src -r -y` was not executed. The
manifest now uses the `armadillo` rosdep key, while CMake continues to use
`find_package(Armadillo REQUIRED)`.

Physical sensors, visual calibration alignment, ROS1 bag replay/conversion, and
interactive RViz rendering were not tested. Synthetic tests do not establish
complete numerical equivalence on real sensor recordings.
