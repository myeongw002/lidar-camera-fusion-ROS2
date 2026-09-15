#!/usr/bin/env python3
"""Exercise installed nodes over DDS; source the built workspace before running."""
import math
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def main():
    prefix = Path(get_package_prefix('lidar_camera_fusion'))
    share = Path(get_package_share_directory('lidar_camera_fusion'))
    executable = prefix / 'lib/lidar_camera_fusion'
    processes = []
    logs = []
    rclpy.init()
    node = rclpy.create_node('fusion_smoke_test')
    try:
        for name, config in [('interpolated_node', 'interpolated'), ('lidar_camera_node', 'fusion')]:
            command = [str(executable / name), '--ros-args', '--params-file', str(share / f'config/{config}.yaml')]
            if name == 'lidar_camera_node':
                command += ['--params-file', str(share / 'config/calibration.yaml')]
            # Keep the test inexpensive, and put our 10 m ring inside the limit.
            command += ['-p', 'maxlen:=20.0', '-p', 'x_resolution:=1.0', '-p', 'filter_output_pc:=false']
            log = tempfile.TemporaryFile(mode='w+')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        time.sleep(2)
        assert all(p.poll() is None for p in processes), 'node failed without data'
        messages = {}
        message_counts = {}
        subscriptions = []
        for topic, kind in [('/pc_interpoled', PointCloud2), ('/pc2imageInterpol', Image),
                            ('/points2', PointCloud2), ('/pcOnImage_image', Image)]:
            def receive(msg, t=topic):
                messages[t] = msg
                message_counts[t] = message_counts.get(t, 0) + 1
            subscriptions.append(node.create_subscription(kind, topic, receive, 10))
        pc_pub = node.create_publisher(PointCloud2, '/velodyne_points', qos_profile_sensor_data)
        img_pub = node.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        header = Header(frame_id='test_lidar')
        header.stamp.sec = 123
        header.stamp.nanosec = 456000
        points = []
        for el in range(-15, 16, 2):
            for az in range(360):
                e, a = math.radians(el), math.radians(az)
                points.append((10*math.cos(e)*math.cos(a), 10*math.cos(e)*math.sin(a), 10*math.sin(e)))
        pc = point_cloud2.create_cloud_xyz32(header, points)
        image = Image()
        image.header.frame_id = 'test_camera'
        image.header.stamp = header.stamp
        image.height, image.width = 720, 1280
        image.encoding, image.step = 'bgr8', 1280*3
        image.data = bytes([17, 83, 201]) * (720*1280)
        deadline = time.monotonic() + 5
        while (pc_pub.get_subscription_count() < 2 or img_pub.get_subscription_count() < 1) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert pc_pub.get_subscription_count() == 2, 'sensor QoS/discovery mismatch'
        # Empty and NaN clouds must not kill either node.
        for values in [[], [(float('nan'), 0.0, 0.0)]]:
            bad = point_cloud2.create_cloud_xyz32(header, values)
            for _ in range(3):
                pc_pub.publish(bad)
                img_pub.publish(image)
                rclpy.spin_once(node, timeout_sec=0.1)
        # A valid but fully range-filtered frame must publish empty cloud frames,
        # and fusion must publish the unchanged camera image with its header.
        messages.clear()
        counts_before = dict(message_counts)
        unusable = point_cloud2.create_cloud_xyz32(header, [(1000.0, 0.0, 0.0)])
        deadline = time.monotonic() + 5
        required_empty = ['/pc_interpoled', '/points2', '/pcOnImage_image']
        while any(message_counts.get(t, 0) == counts_before.get(t, 0) for t in required_empty) and time.monotonic() < deadline:
            pc_pub.publish(unusable)
            img_pub.publish(image)
            rclpy.spin_once(node, timeout_sec=0.1)
        assert messages['/pc_interpoled'].width == 0, 'interpolation dropped/nonempty unusable frame'
        assert messages['/points2'].width == 0, 'fusion dropped/nonempty unusable frame'
        assert messages['/pc_interpoled'].header == header and messages['/points2'].header == header
        assert messages['/pcOnImage_image'].header == image.header
        assert bytes(messages['/pcOnImage_image'].data) == bytes(image.data), 'empty fusion overlay changed'
        messages.clear()
        # Give the valid frame a distinct stamp so queued empty outputs cannot
        # satisfy the valid-output assertions.
        header.stamp.sec = 124
        image.header.stamp.sec = 124
        pc = point_cloud2.create_cloud_xyz32(header, points)
        deadline = time.monotonic() + 15
        while (len(messages) < 4 or
               any(msg.header.stamp.sec != 124 for msg in messages.values())) and time.monotonic() < deadline:
            pc_pub.publish(pc)
            img_pub.publish(image)
            rclpy.spin_once(node, timeout_sec=0.2)
        assert len(messages) == 4, f'missing outputs: {messages.keys()}'
        for topic in ['/pc_interpoled', '/points2', '/pc2imageInterpol']:
            assert messages[topic].header == header, f'wrong LiDAR header: {topic}'
        assert messages['/pcOnImage_image'].header == image.header, 'wrong image header'
        assert messages['/pc2imageInterpol'].encoding == 'mono16'
        assert messages['/pcOnImage_image'].encoding == 'bgr8'
        assert messages['/pc_interpoled'].width > len(points), 'no densification'
        colored = messages['/points2']
        assert colored.width > 0, 'no colored points'
        rows = list(point_cloud2.read_points(colored, field_names=('x', 'y', 'z'), skip_nans=False))
        assert all(math.isfinite(float(v)) for row in rows for v in row), 'nonfinite output'
        rgb_field = next(f for f in colored.fields if f.name == 'rgb')
        packed = struct.unpack_from('>I' if colored.is_bigendian else '<I', colored.data, rgb_field.offset)[0]
        assert packed & 0xFFFFFF == (201 << 16 | 83 << 8 | 17), 'wrong RGB sampling'
        assert bytes(messages['/pcOnImage_image'].data) != bytes(image.data), 'overlay unchanged'
        assert all(p.poll() is None for p in processes), 'node exited during processing'
        print(f'DDS smoke test passed: all four outputs; {colored.width} colored points; headers, RGB, overlay, sensor QoS, invalid/empty inputs')
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
