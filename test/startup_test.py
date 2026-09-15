#!/usr/bin/env python3
"""Check startup validation and the three installed headless launches."""
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml

share = Path(get_package_share_directory('lidar_camera_fusion'))
bin_dir = Path(get_package_prefix('lidar_camera_fusion')) / 'lib/lidar_camera_fusion'
calibration = ['--params-file', str(share / 'config/calibration.yaml')]


def check_calibration_regression():
    values = yaml.safe_load((share / 'config/calibration.yaml').read_text())
    matrix_file = values['/**']['ros__parameters']['matrix_file']
    camera = matrix_file['camera_matrix']
    rotation = matrix_file['rlc']
    translation = matrix_file['tlc']
    assert len(camera) == 12 and camera[10] == 1.0, 'camera projection [2,2] must be +1.0'
    # Known forward LiDAR point: [-y, -z, x, 1] = [0, 0, 10, 1].
    camera_xyz = [rotation[row * 3 + 2] * 10.0 + translation[row] for row in range(3)]
    projected = [sum(camera[row * 4 + col] * (camera_xyz + [1.0])[col]
                     for col in range(4)) for row in range(3)]
    assert projected[2] > 0.0, 'forward LiDAR point must have positive camera depth'
    u, v = projected[0] / projected[2], projected[1] / projected[2]
    assert 0.0 <= u < 1280.0 and 0.0 <= v < 720.0, (u, v)


check_calibration_regression()
cases = [
    ('interpolated_node', ['-p', 'x_resolution:=0.0'], 'x_resolution'),
    ('interpolated_node', ['-p', 'ang_Y_resolution:=-1.0'], 'ang_Y_resolution'),
    ('interpolated_node', ['-p', 'y_interpolation:=0'], 'y_interpolation'),
    ('interpolated_node', ['-p', 'y_interpolation:=2.5'], 'y_interpolation'),
    ('interpolated_node', ['-p', 'maxlen:=0.0'], 'maxlen'),
    ('interpolated_node', ['-p', 'min_ang_FOV:=3.0', '-p', 'max_ang_FOV:=1.0'], 'FOV'),
    ('interpolated_node', ['-p', 'max_var:=-1.0'], 'max_var'),
    ('lidar_camera_node', calibration + ['-p', 'sync_queue_size:=0'], 'sync_queue_size'),
    ('lidar_camera_node', [], 'matrix_file.tlc'),
    ('lidar_camera_node', calibration + ['-p', 'matrix_file.tlc:=[0.0, 1.0]'], 'exactly 3'),
    ('lidar_camera_node', calibration + ['-p', 'matrix_file.rlc:=[1.0]'], 'exactly 9'),
    ('lidar_camera_node', calibration + ['-p', 'matrix_file.camera_matrix:=[1.0]'], 'exactly 12'),
]
for executable, args, expected in cases:
    result = subprocess.run([str(bin_dir / executable), '--ros-args'] + args,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
    assert result.returncode == 1 and expected in result.stdout, (args, result.returncode, result.stdout)
print(f'{len(cases)} invalid-parameter/calibration startup checks passed')
for launch in ['interpolated_vlp16', 'vlp16_on_img', 'vlp16_on_img_offline']:
    command = ['ros2', 'launch', 'lidar_camera_fusion', launch + '.launch.py']
    result = subprocess.run(command + ['--show-args'], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0 and 'pcTopic' in result.stdout, result.stderr
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(command + ['rviz:=false'], stdout=log, stderr=subprocess.STDOUT)
        try:
            time.sleep(2)
            assert process.poll() is None, 'launch exited'
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
        log.seek(0)
        output = log.read()
        assert 'Waiting for' in output and 'process has died' not in output, output
    print(f'{launch}: argument parsing, YAML load, idle startup and shutdown passed')
# Validate RViz classes against installed plugin manifests without requiring a display.
# rviz_common panels are built into RViz rather than pluginlib exports.
default = yaml.safe_load((Path(get_package_share_directory('rviz_common')) / 'default.rviz').read_text())
classes = {panel['Class'] for panel in default['Panels']}
for package in ['rviz_common', 'rviz_default_plugins']:
    for path in Path(get_package_share_directory(package)).rglob('*.xml'):
        try:
            document = ET.parse(path)
        except ET.ParseError:
            continue
        for element in document.iter('class'):
            classes.add(element.attrib.get('name', ''))
def check_classes(value):
    if isinstance(value, dict):
        if value.get('Class'):
            assert value['Class'] in classes, value['Class']
        for v in value.values():
            check_classes(v)
    elif isinstance(value, list):
        for v in value:
            check_classes(v)
for path in (share / 'rviz').glob('*.rviz'):
    check_classes(yaml.safe_load(path.read_text()))
print('RViz YAML parsed; configured classes match installed Humble plugins/built-in panels')


def test_calibration_projection_regression():
    check_calibration_regression()


def test_startup_validation_completed():
    # Startup/launch/RViz checks above execute during pytest collection so any
    # failure aborts this ament test. This marker gives the suite a test result.
    assert True
