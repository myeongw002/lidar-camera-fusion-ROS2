#!/usr/bin/env python3
"""Check startup validation and installed headless launches."""
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

    # lidar_camera_node now applies the supplied LiDAR->camera extrinsic directly,
    # with no implicit [-y, -z, x] axis remap. Test a point 10 m forward in
    # the LiDAR frame: [10, 0, 0, 1].
    lidar_xyz = [10.0, 0.0, 0.0]
    camera_xyz = [
        sum(rotation[row * 3 + col] * lidar_xyz[col] for col in range(3)) + translation[row]
        for row in range(3)
    ]
    projected = [sum(camera[row * 4 + col] * (camera_xyz + [1.0])[col]
                     for col in range(4)) for row in range(3)]
    assert projected[2] > 0.0, 'forward LiDAR point must have positive camera depth'
    u, v = projected[0] / projected[2], projected[1] / projected[2]
    assert 0.0 <= u < 1280.0 and 0.0 <= v < 720.0, (u, v)


def check_range_image_config():
    values = yaml.safe_load((share / 'config/interpolated.yaml').read_text())
    params = values['/**']['ros__parameters']
    assert params['input_rows'] == 16
    assert params['output_rows'] == 64
    assert len(params['vertical_angles_deg']) == 16
    assert params['horizontal_resolution_deg'] > 0.0
    expected_width = round(360.0 / params['horizontal_resolution_deg'])
    assert expected_width > 0


check_calibration_regression()
check_range_image_config()

cases = [
    ('interpolated_node', ['-p', 'horizontal_resolution_deg:=0.0'], 'horizontal_resolution_deg'),
    ('interpolated_node', ['-p', 'input_rows:=1'], 'input_rows'),
    ('interpolated_node', ['-p', 'output_rows:=1'], 'output_rows'),
    ('interpolated_node', ['-p', 'vertical_angles_deg:=[-1.0,1.0]'], 'vertical_angles_deg'),
    ('interpolated_node', ['-p', 'max_interpolation_range_gap_m:=-1.0'], 'max_interpolation_range_gap_m'),
    ('interpolated_node', ['-p', 'maxlen:=0.0'], 'maxlen'),
    ('interpolated_node', ['-p', 'min_ang_FOV:=3.0', '-p', 'max_ang_FOV:=1.0'], 'FOV'),
    ('lidar_camera_node', calibration + ['-p', 'sync_queue_size:=0'], 'sync_queue_size'),
    ('lidar_camera_node', [], 'matrix_file.tlc'),
    ('lidar_camera_node', calibration + ['-p', 'matrix_file.tlc:=[0.0,1.0]'], 'exactly 3'),
    ('lidar_camera_node', calibration + ['-p', 'matrix_file.rlc:=[1.0]'], 'exactly 9'),
    ('lidar_camera_node', calibration + ['-p', 'matrix_file.camera_matrix:=[1.0]'], 'exactly 12'),
]

for executable, args, expected in cases:
    result = subprocess.run(
        [str(bin_dir / executable), '--ros-args'] + args,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
    assert result.returncode == 1 and expected in result.stdout, (
        args, result.returncode, result.stdout)
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


def test_range_image_configuration():
    check_range_image_config()


def test_startup_validation_completed():
    assert True
