# LiDAR–camera fusion — ROS2 Humble

ROS2 Humble port derived from [EPVelasco/lidar-camera-fusion](https://github.com/EPVelasco/lidar-camera-fusion). This branch replaces the upstream PCL `RangeImageSpherical` representation with a fixed-size range image built directly from the Velodyne `ring` field and azimuth.

## Pipeline

Input `/velodyne_points` must be a `sensor_msgs/msg/PointCloud2` containing at least:

- `x`, `y`, `z`: `FLOAT32`
- `ring`: `UINT16`

The range-image path is:

```text
PointCloud2 (x,y,z,ring)
        ↓
ring + azimuth binning
        ↓
raw metric range image: input_rows × W
        ↓
vertical interpolation only
        ↓
interpolated metric range image: output_rows × W
        ↓
spherical XYZ reconstruction
        ↓
interpolated cloud / camera fusion
```

Horizontal width is derived from the configured angular resolution:

```text
W = round(360 / horizontal_resolution_deg)
```

With the default VLP-16 parameters:

```text
input_rows: 16
output_rows: 64
horizontal_resolution_deg: 0.2
```

this gives:

```text
raw range image          = 16 × 1800
interpolated range image = 64 × 1800
```

Both range images are published as `sensor_msgs/msg/Image` with `32FC1` encoding. Values are metric ranges in metres and invalid cells are `NaN`.

## Interpolation

The 16 input rows are identified by the incoming `ring` value. `vertical_angles_deg` defines the physical elevation angle associated with each ring index. The implementation sorts those elevations physically before interpolation, so a non-monotonic ring-to-elevation mapping can also be supplied.

The output rows are uniformly distributed in elevation between the minimum and maximum configured input elevations. Interpolation is vertical only; horizontal columns are never interpolated.

For each target cell, both adjacent measured ring cells must be valid. Interpolation is rejected when:

```text
abs(r_upper - r_lower) > max_interpolation_range_gap_m
```

This prevents obvious foreground/background depth boundaries from being bridged by linear interpolation.

If multiple LiDAR points map to the same `(ring, azimuth-bin)` cell, the nearest range is retained.

XYZ is reconstructed directly from range, elevation, and azimuth:

```text
x = R cos(phi) cos(theta)
y = R cos(phi) sin(theta)
z = R sin(phi)
```

The existing optional ground-angle correction, camera FOV crop, LiDAR-to-camera calibration, RGB sampling, and overlay generation are retained.

## Topics

### Interpolation node

Inputs:

- `/velodyne_points` — `PointCloud2`

Outputs:

- `/range_image_raw` — `Image`, `32FC1`, `input_rows × W`
- `/range_image_interpolated` — `Image`, `32FC1`, `output_rows × W`
- `/pc_interpoled` — interpolated `PointCloud2`

### Fusion node

Inputs:

- `/velodyne_points` — `PointCloud2`
- `/camera/color/image_raw` — `Image`

Outputs:

- `/range_image_raw` — `Image`, `32FC1`
- `/range_image_interpolated` — `Image`, `32FC1`
- `/points2` — camera-colored interpolated `PointCloud2`
- `/pcOnImage_image` — interpolated-cloud projection overlay
- `/pcOnImage_raw_image` — original VLP-16 projection overlay

Topic names are configurable in the YAML files.

## Main parameters

```yaml
maxlen: 100.0
minlen: 0.3
input_rows: 16
output_rows: 64
horizontal_resolution_deg: 0.2
vertical_angles_deg: [-15.0, -13.0, -11.0, -9.0, -7.0, -5.0, -3.0, -1.0,
                       1.0,   3.0,   5.0,  7.0,  9.0, 11.0, 13.0, 15.0]
max_interpolation_range_gap_m: 2.0
```

`horizontal_resolution_deg` is the source of truth for image width; `cols` is not independently configured.

## Dependencies

Ubuntu 22.04 / ROS2 Humble:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-humble-ament-cmake ros-humble-rclcpp ros-humble-sensor-msgs \
  ros-humble-std-msgs ros-humble-cv-bridge ros-humble-message-filters \
  ros-humble-pcl-conversions ros-humble-launch-ros ros-humble-rviz2 \
  libpcl-dev libopencv-dev libeigen3-dev
```

Armadillo and PCL `RangeImageSpherical` are no longer used by this branch.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select lidar_camera_fusion \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

```bash
ros2 launch lidar_camera_fusion interpolated_vlp16.launch.py
ros2 launch lidar_camera_fusion vlp16_on_img.launch.py
ros2 launch lidar_camera_fusion vlp16_on_img_offline.launch.py
```

Sensor drivers are not started by these launches.

## Camera calibration

`config/calibration.yaml` retains the existing camera intrinsic and LiDAR-to-camera extrinsic convention:

```text
p = Mc * T * [-lidar_y, -lidar_z, lidar_x, 1]^T
u = p.x / p.z
v = p.y / p.z
```

Calibrate for the actual sensor mount before relying on visual alignment.

## Validation

```bash
colcon test --packages-select lidar_camera_fusion
colcon test-result --verbose
```

The tests are being updated for the fixed ring range-image path. See `VALIDATION.md` for the validation status of this branch.

## License

GPLv3; see `LICENSE`. Original implementation attribution remains with EPVelasco/lidar-camera-fusion.
