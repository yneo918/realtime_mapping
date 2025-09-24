#!/usr/bin/env python3

import argparse
import math
import random
from copy import deepcopy
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import yaml


def import_message_type(message_type: str):
    """Resolve a ROS 2 message type string to the corresponding class."""
    parts = message_type.split('/')
    if len(parts) < 2:
        raise ValueError(f"Invalid message type format: {message_type}")

    module_name = f"{parts[0]}.msg"
    class_name = parts[-1]

    module = __import__(module_name, fromlist=[class_name])
    return getattr(module, class_name)


def set_nested_value(obj, field_path: str, value):
    target, attr_name = resolve_parent(obj, field_path)
    if isinstance(attr_name, int):
        parent_list = target
        while len(parent_list) <= attr_name:
            parent_list.append(type(value)())
        parent_list[attr_name] = value
    else:
        setattr(target, attr_name, value)


def resolve_parent(obj, field_path: str):
    current = obj
    tokens = tokenize(field_path)
    for token in tokens[:-1]:
        current = ensure_child(current, token)
    last = tokens[-1]
    if isinstance(last, int):
        return current, last
    if isinstance(last, str) and '[' in last:
        raise ValueError(f"Unexpected token: {last}")
    return current, last


def ensure_child(current, token):
    if isinstance(token, int):
        if not isinstance(current, list):
            raise TypeError(f"Expected list for index access, got {type(current)}")
        while len(current) <= token:
            current.append(deepcopy(current[0]) if current else 0.0)
        return current[token]

    if isinstance(token, str) and token.endswith(']') and '[' in token:
        base, _, index_part = token.partition('[')
        index = int(index_part.rstrip(']'))
        child = getattr(current, base)
        if not isinstance(child, list):
            raise TypeError(f"Expected list in attribute {base}")
        while len(child) <= index:
            child.append(deepcopy(child[0]) if child else 0.0)
        return child[index]

    return getattr(current, token)


def tokenize(field_path: str):
    tokens = []
    parts = field_path.split('.')
    for part in parts:
        while '[' in part:
            before, _, rest = part.partition('[')
            if before:
                tokens.append(before)
            index_str, _, remainder = rest.partition(']')
            tokens.append(int(index_str))
            part = remainder.lstrip('.')
        if part:
            tokens.append(part)
    return tokens


class PublisherPair:
    """Helper that publishes one set of position and sensor data."""

    def __init__(
        self,
        node: 'FakePublisher',
        index: int,
        total_pairs: int,
        input_config: Dict[str, Any],
        mapping_origin: Dict[str, float],
        default_motion: Dict[str, float],
        default_sensor_center: float,
        default_sensor_amplitude: float,
    ):
        self.node = node
        self.index = index
        self.name = input_config.get('name', f"input_{index + 1}")

        self.position_config = input_config.get('position')
        self.sensor_config = input_config.get('sensor_data')
        if not self.position_config or not self.sensor_config:
            raise ValueError('position/sensor configuration is required')

        self.position_msg_type = import_message_type(self.position_config['message_type'])
        self.sensor_msg_type = import_message_type(self.sensor_config['message_type'])

        self.position_publisher = node.create_publisher(
            self.position_msg_type,
            self.position_config['topic'],
            10
        )
        self.sensor_publisher = node.create_publisher(
            self.sensor_msg_type,
            self.sensor_config['topic'],
            10
        )

        self.position_fields = self.position_config.get('fields', {})
        self.sensor_fields = self.sensor_config.get('fields', {})
        self.uses_navsat = self.position_config.get('message_type') == 'sensor_msgs/NavSatFix'

        simulation_cfg = input_config.get('simulation', {})
        motion_cfg = simulation_cfg.get('motion', {})

        self.rate = motion_cfg.get('rate', default_motion['rate'])
        self.radius = motion_cfg.get('radius', default_motion['radius'])
        self.noise = motion_cfg.get('noise', default_motion['noise'])
        self.angular_speed = motion_cfg.get('angular_speed', default_motion['angular_speed'])

        if 'phase_rad' in motion_cfg:
            self.phase = float(motion_cfg['phase_rad'])
        elif 'phase_deg' in motion_cfg:
            self.phase = math.radians(motion_cfg['phase_deg'])
        else:
            self.phase = index * (2 * math.pi / max(total_pairs, 1))

        self.value_center = simulation_cfg.get('value_center', default_sensor_center)
        self.value_amplitude = simulation_cfg.get('value_amplitude', default_sensor_amplitude)

        if self.uses_navsat:
            geo_origin_cfg = self.position_config.get('geo_origin', {})
            self.base_lat = geo_origin_cfg.get('latitude', mapping_origin.get('y', 0.0))
            self.base_lon = geo_origin_cfg.get('longitude', mapping_origin.get('x', 0.0))
            geo_offset = simulation_cfg.get('geo_origin_offset', {})
            self.base_lat += geo_offset.get('latitude', 0.0)
            self.base_lon += geo_offset.get('longitude', 0.0)
            reference_lat = self.base_lat
            self.longitude_scale = 111320.0 * math.cos(math.radians(reference_lat))
            self.latitude_scale = 110540.0
        else:
            origin_cfg = self.position_config.get('origin', {})
            self.base_x = origin_cfg.get('x', mapping_origin.get('x', 0.0))
            self.base_y = origin_cfg.get('y', mapping_origin.get('y', 0.0))

        self.publish_interval = 1.0 / max(self.rate, 1e-3)
        self.start_time: Time = node.get_clock().now()
        self.next_publish_time = self.start_time.nanoseconds / 1e9

        node.get_logger().info(
            f"Pair {self.name}: position->{self.position_config['topic']}, sensor->{self.sensor_config['topic']}, "
            f"rate={self.rate:.2f}Hz"
        )

    def update(self):
        now = self.node.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        if now_sec + 1e-9 < self.next_publish_time:
            return

        while now_sec + 1e-9 >= self.next_publish_time:
            self._publish_once(now)
            self.next_publish_time += self.publish_interval
            now = self.node.get_clock().now()
            now_sec = now.nanoseconds / 1e9

    def _publish_once(self, now: Time):
        elapsed = (now - self.start_time).nanoseconds / 1e9
        angle = elapsed * self.angular_speed + self.phase
        offset_x = self.radius * math.cos(angle)
        offset_y = self.radius * math.sin(angle)

        position_msg = self.position_msg_type()
        sensor_msg = self.sensor_msg_type()

        timestamp_msg = now.to_msg()

        if hasattr(position_msg, 'header'):
            position_msg.header.stamp = timestamp_msg
            position_msg.header.frame_id = 'map'

        if self.uses_navsat:
            lon_scale = self.longitude_scale if self.longitude_scale else 1.0
            lat_scale = self.latitude_scale if self.latitude_scale else 1.0
            position_msg.latitude = self.base_lat + (offset_y / lat_scale)
            position_msg.longitude = self.base_lon + (offset_x / lon_scale if lon_scale else 0.0)
            position_msg.altitude = 0.0
        else:
            world_x = self.base_x + offset_x
            world_y = self.base_y + offset_y
            self._assign_position_fields(position_msg, world_x, world_y)

        sensor_value = self.value_center + self.value_amplitude * math.sin(angle)
        sensor_value += random.uniform(-self.noise, self.noise)
        self._assign_sensor_value(sensor_msg, sensor_value)

        if hasattr(sensor_msg, 'header'):
            sensor_msg.header.stamp = timestamp_msg
            sensor_msg.header.frame_id = 'map'

        self.position_publisher.publish(position_msg)
        self.sensor_publisher.publish(sensor_msg)

    def _assign_position_fields(self, msg, x: float, y: float):
        x_path = self.position_fields.get('x')
        y_path = self.position_fields.get('y')
        if not x_path or not y_path:
            self.node.get_logger().warn(f"{self.name}: position fields not configured; skip publish")
            return
        set_nested_value(msg, x_path, x)
        set_nested_value(msg, y_path, y)
        z_path = self.position_fields.get('z')
        if z_path:
            set_nested_value(msg, z_path, 0.0)

    def _assign_sensor_value(self, msg, value: float):
        value_path = self.sensor_fields.get('value')
        if not value_path:
            self.node.get_logger().warn(f"{self.name}: sensor value field not configured; skip publish")
            return
        set_nested_value(msg, value_path, float(value))


class FakePublisher(Node):
    """Publisher that simulates every input defined in the configuration."""

    def __init__(self, config: Dict[str, Any], rate_hz: float, radius: float, noise: float):
        super().__init__('fake_publisher')

        self.config = config
        mapping_origin = self.config.get('mapping', {}).get('origin', {'x': 0.0, 'y': 0.0})

        sensor_defaults = self.config.get('sensor_defaults', {})
        value_range = sensor_defaults.get('value_range', {})
        sensor_min = value_range.get('min', 0.0)
        sensor_max = value_range.get('max', 100.0)

        default_motion = {
            'rate': rate_hz,
            'radius': radius,
            'noise': noise,
            'angular_speed': math.pi / 15.0,
        }
        default_sensor_center = (sensor_min + sensor_max) / 2.0
        default_sensor_amplitude = (sensor_max - sensor_min) / 3.0 if sensor_max > sensor_min else 30.0

        inputs_config = self.config.get('inputs', [])
        if not inputs_config:
            raise ValueError('No inputs defined in configuration')

        enabled_inputs = [cfg for cfg in inputs_config if cfg.get('simulation', {}).get('enabled', True)]
        if not enabled_inputs:
            self.get_logger().warn('No inputs enabled for simulation; FakePublisher will idle')

        self.publisher_pairs = []
        intervals = []

        total_pairs = len(enabled_inputs)
        for idx, input_cfg in enumerate(enabled_inputs):
            try:
                pair = PublisherPair(
                    node=self,
                    index=idx,
                    total_pairs=total_pairs,
                    input_config=input_cfg,
                    mapping_origin=mapping_origin,
                    default_motion=default_motion,
                    default_sensor_center=default_sensor_center,
                    default_sensor_amplitude=default_sensor_amplitude,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to initialize publisher for {input_cfg.get('name', f'input_{idx + 1}')}: {exc}"
                )
                continue

            self.publisher_pairs.append(pair)
            intervals.append(pair.publish_interval)

        if self.publisher_pairs:
            min_interval = min(intervals)
            self.timer_period = max(0.01, min_interval / 2.0)
            self.master_timer = self.create_timer(self.timer_period, self._on_timer)
            self.get_logger().info(
                f"Initialized {len(self.publisher_pairs)} publisher pair(s); timer period {self.timer_period:.3f}s"
            )
        else:
            self.master_timer = None
            self.get_logger().warn('No publisher pairs active; nothing will be published')

    def _on_timer(self):
        for pair in self.publisher_pairs:
            pair.update()


def load_config(path: str) -> Dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description='Fake publisher for realtime_mapping tests')
    parser.add_argument('--config', '-c', default='config/message_config.yaml', help='Path to the shared configuration file')
    parser.add_argument('--rate', '-r', type=float, default=5.0, help='Default publishing rate in Hz (per input unless overridden)')
    parser.add_argument('--radius', type=float, default=5.0, help='Default radius for the circular trajectory in meters')
    parser.add_argument('--noise', type=float, default=2.0, help='Default random noise added to sensor values')

    args = parser.parse_args()

    config = load_config(args.config)

    rclpy.init()
    node = FakePublisher(config, args.rate, args.radius, args.noise)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
