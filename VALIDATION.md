# Local validation — 2026-09-14

Environment: Ubuntu 22.04.5, ROS2 Humble, PCL 1.12.1, OpenCV 4.5.4,
Eigen 3.4.0, Armadillo 10.8.2. Upstream source revision:
`621de50a9e0d9c360e4ce9f84b0fc16acbcc5918`.

## Build

The provided directory was empty (including no usable Git metadata). The
upstream repository was cloned into `/tmp/lidar-camera-fusion-original` for
inspection, and the port was written into the requested workspace directory.
Build/install/log files were kept under `/tmp` to avoid modifying the parent
workspace or its existing installation.

Exact command:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/lidar-fusion-validation/log build \
  --base-paths /home/hrilab/ROS2/compa_ws/src/llidar_camera_fusion \
  --build-base /tmp/lidar-fusion-validation/build \
  --install-base /tmp/lidar-fusion-validation/install \
  --symlink-install --packages-select lidar_camera_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result: successful Release build. The initial build exposed ambiguous PCL
`getPoint` overload resolution with Armadillo indices; explicit signed casts
fixed it. The successful build emitted no compiler warnings.

## Executed checks

```bash
source /tmp/lidar-fusion-validation/install/setup.bash
ros2 pkg executables lidar_camera_fusion
colcon --log-base /tmp/lidar-fusion-validation/log test \
  --build-base /tmp/lidar-fusion-validation/build \
  --install-base /tmp/lidar-fusion-validation/install \
  --packages-select lidar_camera_fusion --event-handlers console_direct+
colcon --log-base /tmp/lidar-fusion-validation/log test-result \
  --test-result-base /tmp/lidar-fusion-validation/build --verbose
ROS_LOG_DIR=/tmp/lidar-fusion-validation/ros_logs ROS_DOMAIN_ID=87 \
  ROS_LOCALHOST_ONLY=1 python3 test/ros_smoke_test.py
ROS_LOG_DIR=/tmp/lidar-fusion-validation/ros_logs ROS_DOMAIN_ID=88 \
  ROS_LOCALHOST_ONLY=1 python3 test/startup_test.py
```

- Both `interpolated_node` and `lidar_camera_node` registered as package executables.
- CTest: 1 test passed, 0 failures. Synthetic geometry checks exercise empty and
  nonfinite input, vertical densification, constant-range reconstruction, FOV,
  ground correction, filtering, and invalid settings.
- DDS test: both nodes idle without data and process best-effort synthetic
  VLP16-like rings plus a BGR camera image. All four output topics received;
  7,930 colored points. Verified LiDAR/camera headers, output encodings, finite
  XYZ, RGB channel sampling, and image overlay changes. Nodes survived empty,
  NaN, and fully range-filtered clouds.
- Startup test: invalid angular resolutions, interpolation factor/type, range,
  FOV, variance threshold, sync queue, missing calibration and wrong lengths
  for all three arrays rejected clearly (12 cases).
- All three launch files parse arguments, load their YAML, start headlessly,
  wait without sensor data and shut down on SIGINT.
- RViz YAML and configured classes are checked against installed Humble
  plugin manifests and built-in panels. Interactive GUI rendering was not tested.

The sandbox emitted network-interface/socket permission warnings during DDS
testing, though shared-memory delivery succeeded. The DDS smoke test was also
run outside the sandbox with approval and passed without those warnings.
Warnings for intentionally empty/unusable point clouds are expected.

`rosdep` is not installed in this environment, so the documented `rosdep install`
command was not exercised. Required development libraries were already installed
and found by CMake. No physical sensors, ROS1 bag replay, or visual calibration
alignment were tested. Synthetic tests establish basic behavior, not complete
numerical equivalence on real recordings.
