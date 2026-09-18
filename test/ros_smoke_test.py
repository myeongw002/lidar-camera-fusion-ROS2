#!/usr/bin/env python3
"""Exercise the installed interpolation -> fusion pipeline over DDS."""
import math
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from ament_index_python.packages import get_package_prefix, get_package_share_directory


VERTICAL_ANGLES = [-15.0, -13.0, -11.0, -9.0, -7.0, -5.0, -3.0, -1.0,
                    1.0,   3.0,   5.0,  7.0,  9.0, 11.0, 13.0, 15.0]


def ring_cloud(header, points):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=12, datatype=PointField.UINT16, count=1),
    ]
    return point_cloud2.create_cloud(header, fields, points)


def synthetic_vlp16(header, radius=10.0):
    points = []
    for ring, elevation_deg in enumerate(VERTICAL_ANGLES):
        elevation = math.radians(elevation_deg)
        for azimuth_deg in range(360):
            azimuth = math.radians(azimuth_deg)
            points.append((
                radius * math.cos(elevation) * math.cos(azimuth),
                radius * math.cos(elevation) * math.sin(azimuth),
                radius * math.sin(elevation),
                ring,
            ))
    return ring_cloud(header, points)


def image_float_values(msg):
    endian = '>' if msg.is_bigendian else '<'
    count = msg.height * msg.width
    return struct.unpack(endian + 'f' * count, bytes(msg.data[:count * 4]))


def main():
    prefix = Path(get_package_prefix('lidar_camera_fusion'))
    share = Path(get_package_share_directory('lidar_camera_fusion'))
    executable = prefix / 'lib/lidar_camera_fusion'
    processes = []
    logs = []

    rclpy.init()
    node = rclpy.create_node('fusion_smoke_test')
    try:
        interpolation_command = [
            str(executable / 'interpolated_node'), '--ros-args',
            '--params-file', str(share / 'config/interpolated.yaml'),
            '-p', 'maxlen:=20.0',
            '-p', 'horizontal_resolution_deg:=1.0',
            '-p', 'max_interpolation_range_gap_m:=2.0',
        ]
        fusion_command = [
            str(executable / 'lidar_camera_node'), '--ros-args',
            '--params-file', str(share / 'config/fusion.yaml'),
            '--params-file', str(share / 'config/calibration.yaml'),
            '-p', 'overlay_max_range_m:=20.0',
        ]

        for command in [interpolation_command, fusion_command]:
            log = tempfile.TemporaryFile(mode='w+')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))

        time.sleep(2)
        assert all(p.poll() is None for p in processes), 'node failed without data'

        messages = {}
        message_counts = {}
        subscriptions = []
        topics = [
            ('/pc_interpoled', PointCloud2),
            ('/range_image_raw', Image),
            ('/range_image_interpolated', Image),
            ('/points2', PointCloud2),
            ('/pcOnImage_image', Image),
            ('/pcOnImage_raw_image', Image),
        ]
        for topic, kind in topics:
            def receive(msg, t=topic):
                messages[t] = msg
                message_counts[t] = message_counts.get(t, 0) + 1
            subscriptions.append(node.create_subscription(kind, topic, receive, 10))

        pc_pub = node.create_publisher(PointCloud2, '/velodyne_points', qos_profile_sensor_data)
        img_pub = node.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)

        header = Header(frame_id='test_lidar')
        header.stamp.sec = 123
        header.stamp.nanosec = 456000

        image = Image()
        image.header.frame_id = 'test_camera'
        image.header.stamp = header.stamp
        image.height, image.width = 720, 1280
        image.encoding, image.step = 'bgr8', 1280 * 3
        image.data = bytes([17, 83, 201]) * (720 * 1280)

        deadline = time.monotonic() + 5
        while (pc_pub.get_subscription_count() < 1 or img_pub.get_subscription_count() < 1) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert pc_pub.get_subscription_count() == 1, 'raw LiDAR should be consumed only by interpolation node'
        assert img_pub.get_subscription_count() == 1, 'camera QoS/discovery mismatch'

        # Fully range-filtered input still propagates through the serial pipeline.
        messages.clear()
        counts_before = dict(message_counts)
        unusable = ring_cloud(header, [(1000.0, 0.0, 0.0, 0)])
        required = ['/pc_interpoled', '/range_image_raw', '/range_image_interpolated',
                    '/points2', '/pcOnImage_image', '/pcOnImage_raw_image']
        deadline = time.monotonic() + 5
        while any(message_counts.get(t, 0) == counts_before.get(t, 0) for t in required) and time.monotonic() < deadline:
            pc_pub.publish(unusable)
            img_pub.publish(image)
            rclpy.spin_once(node, timeout_sec=0.1)

        assert all(t in messages for t in required), f'missing unusable-frame outputs: {messages.keys()}'
        assert messages['/pc_interpoled'].width == 0
        assert messages['/points2'].width == 0
        assert messages['/range_image_raw'].height == 16
        assert messages['/range_image_raw'].width == 360
        assert messages['/range_image_interpolated'].height == 64
        assert messages['/range_image_interpolated'].width == 360
        assert messages['/range_image_raw'].encoding == '32FC1'
        assert messages['/range_image_interpolated'].encoding == '32FC1'
        assert all(math.isnan(v) for v in image_float_values(messages['/range_image_raw']))
        assert all(math.isnan(v) for v in image_float_values(messages['/range_image_interpolated']))
        assert messages['/pc_interpoled'].header == header
        assert messages['/points2'].header == header
        assert messages['/pcOnImage_image'].header == image.header
        assert messages['/pcOnImage_raw_image'].header == image.header
        assert bytes(messages['/pcOnImage_image'].data) == bytes(image.data)
        assert bytes(messages['/pcOnImage_raw_image'].data) == bytes(image.data)

        # Valid scan. The fusion node must consume /pc_interpoled rather than raw LiDAR.
        messages.clear()
        header.stamp.sec = 124
        image.header.stamp.sec = 124
        pc = synthetic_vlp16(header)
        deadline = time.monotonic() + 15
        while (len(messages) < len(required) or
               any(msg.header.stamp.sec != 124 for msg in messages.values())) and time.monotonic() < deadline:
            pc_pub.publish(pc)
            img_pub.publish(image)
            rclpy.spin_once(node, timeout_sec=0.2)

        assert all(t in messages for t in required), f'missing outputs: {messages.keys()}'
        for topic in ['/pc_interpoled', '/range_image_raw', '/range_image_interpolated', '/points2']:
            assert messages[topic].header == header, f'wrong LiDAR header: {topic}'
        assert messages['/pcOnImage_image'].header == image.header
        assert messages['/pcOnImage_raw_image'].header == image.header

        raw = messages['/range_image_raw']
        dense = messages['/range_image_interpolated']
        assert (raw.height, raw.width, raw.encoding) == (16, 360, '32FC1')
        assert (dense.height, dense.width, dense.encoding) == (64, 360, '32FC1')
        raw_values = image_float_values(raw)
        dense_values = image_float_values(dense)
        assert all(math.isfinite(v) and abs(v - 10.0) < 1e-3 for v in raw_values)
        assert all(math.isfinite(v) and abs(v - 10.0) < 1e-3 for v in dense_values)

        assert messages['/pc_interpoled'].width == 64 * 360, 'wrong dense cloud size'
        colored = messages['/points2']
        assert colored.width > 0, 'no colored points'
        rows = list(point_cloud2.read_points(colored, field_names=('x', 'y', 'z'), skip_nans=False))
        assert all(math.isfinite(float(v)) for row in rows for v in row), 'nonfinite output'

        rgb_field = next(f for f in colored.fields if f.name == 'rgb')
        packed = struct.unpack_from('>I' if colored.is_bigendian else '<I', colored.data, rgb_field.offset)[0]
        assert packed & 0xFFFFFF == (201 << 16 | 83 << 8 | 17), 'wrong RGB sampling'
        assert bytes(messages['/pcOnImage_image'].data) != bytes(image.data), 'interpolated overlay unchanged'
        assert bytes(messages['/pcOnImage_raw_image'].data) != bytes(image.data), 'raw overlay unchanged'
        assert all(p.poll() is None for p in processes), 'node exited during processing'

        print(
            f'DDS serial-pipeline smoke test passed: raw={raw.height}x{raw.width}, '
            f'dense={dense.height}x{dense.width}, colored={colored.width}')
    finally:
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait()
        for log in logs:
            log.seek(0)
            print(log.read())
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


def test_ros_smoke():
    main()
