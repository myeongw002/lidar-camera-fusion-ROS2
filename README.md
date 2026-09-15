# LiDAR–camera fusion — ROS2 Humble

Native C++17 / `ament_cmake` port of [EPVelasco/lidar-camera-fusion](https://github.com/EPVelasco/lidar-camera-fusion), based on commit `621de50a9e0d9c360e4ce9f84b0fc16acbcc5918`. The supplied workspace was empty; source was inspected from that upstream revision. Licensed under GPLv3; see LICENSE.

## Dependencies

Ubuntu 22.04 and ROS2 Humble:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-humble-ament-cmake ros-humble-rclcpp ros-humble-sensor-msgs \
  ros-humble-std-msgs ros-humble-cv-bridge ros-humble-message-filters \
  ros-humble-pcl-conversions ros-humble-launch-ros ros-humble-rviz2 \
  libpcl-dev libopencv-dev libeigen3-dev libarmadillo-dev
```

No ROS1, `ros1_bridge`, `pcl_ros`, TF calibration system, or sensor drivers are required by this package.

## Build

Place this directory in `~/ros2_ws/src/lidar_camera_fusion`:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select lidar_camera_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 pkg executables lidar_camera_fusion
```

The executables are `interpolated_node` and `lidar_camera_node`.

## Run

Start sensor publishers separately, then:

```bash
ros2 launch lidar_camera_fusion interpolated_vlp16.launch.py
ros2 launch lidar_camera_fusion vlp16_on_img.launch.py
ros2 launch lidar_camera_fusion vlp16_on_img_offline.launch.py
```

Use `rviz:=false` for headless operation. RViz defaults on for interpolation and offline fusion, off for live fusion, as upstream. The fixed frame in RViz is `velodyne`; change it to match your input cloud frame. No external driver launch filenames are assumed.

```bash
ros2 launch lidar_camera_fusion vlp16_on_img.launch.py \
  pcTopic:=/lidar/points imgTopic:=/rgb/image_raw rviz:=true
```

Launch arguments include `params_file`, `use_sim_time`, and (fusion only) `calibration_file`. Offline launch sets processing defaults only; it does not start bag playback. Enable `use_sim_time:=true` when playing rosbag2 with `--clock`. Original upstream `.bag` examples are **ROS1 bags** and require conversion or a separately supported reader; they are not native rosbag2 recordings and have not been tested here.

| Node | Inputs | Outputs |
|---|---|---|
| Interpolation | `/velodyne_points` (PointCloud2) | `/pc_interpoled` (PointCloud2 XYZ), `/pc2imageInterpol` (mono16 visualization) |
| Fusion | `/velodyne_points` (PointCloud2), `/camera/color/image_raw` (Image) | `/points2` (PointCloud2 XYZRGB), `/pcOnImage_image` (bgr8 Image) |

Inputs use best-effort, volatile sensor-data QoS. Fusion uses ApproximateTime with `sync_queue_size: 10`; sensor timestamps must share a clock. Outputs are reliable and volatile. Clouds and range images carry the LiDAR input stamp/frame; overlays carry the camera header. Ground correction is applied to coordinates as upstream; no TF is generated.

For a valid callback whose LiDAR cloud has no usable points, each node publishes an empty PointCloud2 with the LiDAR header. Fusion also publishes the unchanged camera image with its original header. The interpolation node omits the range image only when no usable range-image dimensions exist.

The filter subscribers use the [Humble Subscriber QoS API](https://docs.ros.org/en/ros2_packages/humble/api/message_filters/generated/classmessage__filters_1_1SubscriberBase.html).

## Parameters and calibration

Parameters are node-local, declared and read-only after startup. Edit the YAML or pass startup overrides. Angular resolutions `x_resolution` and `ang_Y_resolution` are in degrees; `ang_ground`, `min_ang_FOV`, and `max_ang_FOV` are radians. FOV accepts `-pi/2 <= min < max <= 3*pi/2`, and tests reconstructed azimuth against `[min-pi/2, max-pi/2]`. `minlen` and `maxlen` are meters, with `0 <= minlen < maxlen`. `y_interpolation` is a positive **integer** (write `10`, not `10.0`). `max_var` is finite and nonnegative. Topics are configurable via `pcTopic`, `imgTopic`, `output_cloud_topic`, and `output_image_topic`.

`config/interpolated.yaml`, `fusion.yaml`, and `fusion_offline.yaml` retain their upstream launch defaults. Fusion now honors `ang_ground`; its default remains the actual upstream hard-coded 0.6 degrees, including offline mode (where upstream ignored a zero-valued launch parameter).

`config/calibration.yaml` retains all upstream values, including `camera_matrix[2,2] = +1.0`. Under `matrix_file`, `tlc` has 3 doubles, `rlc` has 9 row-major doubles, and `camera_matrix` has 12 row-major doubles. Fusion requires these arrays at startup and rejects missing, malformed, or nonfinite values:

```bash
ros2 run lidar_camera_fusion lidar_camera_node --ros-args \
  --params-file /path/to/config/fusion.yaml \
  --params-file /path/to/config/calibration.yaml
```

The convention remains `p = Mc * T * [-lidar_y, -lidar_z, lidar_x, 1]^T`, with `T = [Rlc, tlc; 0,0,0,1]`, then `u=p.x/p.z`, `v=p.y/p.z`. Upstream Eigen comma initialization filled rows; `Rlc(0),Rlc(3),Rlc(6)` retrieved its first row from column-major storage. Direct `(row,col)` assignment is equivalent and does not transpose/invert calibration. Calibrate for your actual camera and image resolution before relying on alignment.

## Algorithm and migration details

Spherical PCL range images, Armadillo vertical interpolation, invalid-neighborhood masking, FOV, ground correction, RGB sampling and distance-colored overlays are retained. The interpolation node computes standard deviation on masked ranges; fusion computes an unnormalized sum of squared deviations on the original interpolated ranges. These different upstream statistics, the last-five-column exclusion, and final-block omission are deliberately preserved.

Definite fixes: `lineal` → `linear`; explicit Armadillo linkage; signed/finite projection bounds and positive depth; finite point/range/interpolation checks; guarded reconstruction square root; safe empty/small images and unsigned loop bounds; validated parameters/calibration; input metadata propagation. Fusion uses XYZ internally because upstream intensity was never used. Very small negative square-root arguments are clamped; invalid points are skipped. Negative fractional pixel coordinates are rejected before truncation. Overlay channels are clamped to byte range. The legacy mono16 range visualization uses explicit truncation/modulo conversion to replace undefined negative float-to-unsigned casts; it is **not metric depth**.

## Validation

```bash
colcon test --packages-select lidar_camera_fusion
colcon test-result --verbose
```

Normal `colcon test` runs the C++ interpolation test and both Python runtime suites through `ament_cmake_pytest`. These cover synthetic rings, densification, range preservation, FOV, ground correction, empty/nonfinite inputs, empty output frames, filtering, parameter/calibration rejection, calibration projection depth, DDS/QoS, headers, RGB sampling, overlays, launch startup and RViz2 configuration. See `VALIDATION.md` for commands and actual local results. Real hardware alignment and ROS1 bag replay are not claimed.
