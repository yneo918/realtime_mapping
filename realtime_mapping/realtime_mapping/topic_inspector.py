#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import json
import argparse
from typing import Dict, List, Tuple
import importlib
import yaml
import os


class TopicInspector(Node):
    def __init__(self):
        super().__init__('topic_inspector')

    def get_topic_list(self) -> List[Tuple[str, str]]:
        """Retrieve available topics (excluding common infrastructure topics)."""
        try:
            result = subprocess.run(['ros2', 'topic', 'list', '-t'],
                                  capture_output=True, text=True, check=True)
            topics = []
            for line in result.stdout.strip().split('\n'):
                if line and not any(excluded in line for excluded in [
                    '/rosout', '/parameter_events', '/tf', '/tf_static'
                ]):
                    parts = line.split(' ')
                    if len(parts) >= 2:
                        topic_name = parts[0]
                        message_type = parts[1].strip('[]')
                        topics.append((topic_name, message_type))
            return topics
        except Exception as e:
            self.get_logger().error(f"Failed to get topics: {e}")
            return []

    def inspect_message_structure(self, message_type: str) -> Dict:
        """Inspect the structure of a given ROS 2 message type."""
        try:
            # Obtain the message definition from ROS 2
            result = subprocess.run(['ros2', 'interface', 'show', message_type],
                                  capture_output=True, text=True, check=True)

            structure = {
                'message_type': message_type,
                'definition': result.stdout,
                'fields': self.parse_message_fields(result.stdout)
            }
            return structure
        except Exception as e:
            self.get_logger().error(f"Failed to inspect message {message_type}: {e}")
            return {}

    def parse_message_fields(self, definition: str) -> List[Dict]:
        """Parse field metadata from a raw message definition."""
        fields = []
        lines = definition.strip().split('\n')

        for line in lines:
            line = line.strip()
            if line and not line.startswith('#') and ' ' in line:
                parts = line.split()
                if len(parts) >= 2:
                    field_type = parts[0]
                    field_name = parts[1]

                    # Determine whether the field is an array
                    is_array = '[]' in field_type
                    base_type = field_type.replace('[]', '')

                    fields.append({
                        'name': field_name,
                        'type': field_type,
                        'base_type': base_type,
                        'is_array': is_array
                    })

        return fields

    def generate_config_template(self, position_topic: Tuple[str, str],
                               sensor_topic: Tuple[str, str]) -> Dict:
        """Build a configuration template using the selected topics."""
        pos_topic_name, pos_msg_type = position_topic
        sensor_topic_name, sensor_msg_type = sensor_topic

        # Parse message structures
        pos_structure = self.inspect_message_structure(pos_msg_type)
        sensor_structure = self.inspect_message_structure(sensor_msg_type)

        # Guess mapping fields for both feeds
        pos_fields = self.guess_position_fields(pos_structure.get('fields', []))
        sensor_fields = self.guess_sensor_fields(sensor_structure.get('fields', []))

        config = {
            'position': {
                'topic': pos_topic_name,
                'message_type': pos_msg_type,
                'fields': pos_fields
            },
            'sensor_data': {
                'topic': sensor_topic_name,
                'message_type': sensor_msg_type,
                'fields': sensor_fields,
                'value_range': {'min': 0.0, 'max': 100.0},
                'colormap': 'hot'
            },
            'mapping': {
                'cell_size': 1.0,
                'map_size': {'width': 100, 'height': 100},
                'origin': {'x': 0.0, 'y': 0.0},
                'update_method': 'average',
                'display': {
                    'update_rate': 10.0,
                    'colormap': 'viridis',
                    'show_grid': True,
                    'show_colorbar': True
                }
            },
            'output': {
                'csv': {
                    'enabled': True,
                    'filename_pattern': 'mapping_data_{timestamp}.csv',
                    'include_metadata': True
                },
                'image': {
                    'enabled': True,
                    'filename_pattern': 'heatmap_{timestamp}.png',
                    'dpi': 300,
                    'format': 'png'
                }
            }
        }

        return config

    def guess_position_fields(self, fields: List[Dict]) -> Dict:
        """Infer likely position-related field paths."""
        pos_fields = {}

        # Check well-known field names first
        field_names = [f['name'] for f in fields]

        if 'latitude' in field_names and 'longitude' in field_names:
            # GPS coordinates
            pos_fields = {
                'latitude': 'latitude',
                'longitude': 'longitude'
            }
            if 'altitude' in field_names:
                pos_fields['altitude'] = 'altitude'
        else:
            # Cartesian coordinates
            direct_axes = {'x', 'y', 'z'}
            if all(axis in field_names for axis in ['x', 'y']):
                for axis in direct_axes:
                    if axis in field_names:
                        pos_fields[axis] = axis
            else:
                for field in fields:
                    if field['name'] == 'pose':
                        base_type = field.get('base_type', '')
                        if 'PoseWithCovariance' in base_type:
                            pos_fields = {
                                'x': 'pose.pose.position.x',
                                'y': 'pose.pose.position.y',
                                'z': 'pose.pose.position.z'
                            }
                        else:
                            pos_fields = {
                                'x': 'pose.position.x',
                                'y': 'pose.position.y',
                                'z': 'pose.position.z'
                            }
                        break
                    if field['name'] == 'position':
                        pos_fields = {
                            'x': 'position.x',
                            'y': 'position.y',
                            'z': 'position.z'
                        }
                        break

        return pos_fields

    def guess_sensor_fields(self, fields: List[Dict]) -> Dict:
        """Infer likely sensor value field paths."""
        sensor_fields = {}

        # Check common field names
        for field in fields:
            if field['name'] in ['data', 'value', 'intensity', 'distance']:
                sensor_fields['value'] = field['name']
                break
            elif field['is_array'] and 'intensities' in field['name']:
                # Array data such as LaserScan
                sensor_fields['value'] = f"{field['name']}[0]"
                break

        return sensor_fields

    def interactive_selection(self) -> Tuple[Tuple[str, str], Tuple[str, str]]:
        """Interactively select position and sensor topics."""
        topics = self.get_topic_list()

        if not topics:
            self.get_logger().error("No topics found")
            return None, None

        print("\nAvailable topics:")
        for i, (topic, msg_type) in enumerate(topics):
            print(f"{i+1:2d}: {topic:<30} ({msg_type})")

        # Choose a position topic
        while True:
            try:
                choice = input("\nSelect position topic (number): ")
                pos_idx = int(choice) - 1
                if 0 <= pos_idx < len(topics):
                    position_topic = topics[pos_idx]
                    break
                else:
                    print("Invalid selection. Please try again.")
            except (ValueError, KeyboardInterrupt):
                print("Invalid selection. Please try again.")

        # Choose a sensor topic
        while True:
            try:
                choice = input("Select sensor topic (number): ")
                sensor_idx = int(choice) - 1
                if 0 <= sensor_idx < len(topics):
                    sensor_topic = topics[sensor_idx]
                    break
                else:
                    print("Invalid selection. Please try again.")
            except (ValueError, KeyboardInterrupt):
                print("Invalid selection. Please try again.")

        return position_topic, sensor_topic

    def save_config(self, config: Dict, filename: str = None):
        """Persist a generated configuration template to disk."""
        if filename is None:
            filename = 'config/generated_config.yaml'

        directory = os.path.dirname(os.path.abspath(filename))
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)

        with open(filename, 'w', encoding='utf-8') as f:
            yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

        self.get_logger().info(f"Configuration saved to {filename}")

    def print_topic_info(self, topics: List[Tuple[str, str]]):
        """Pretty-print topic metadata and a preview of their fields."""
        print(f"\nFound {len(topics)} topics:")
        print("-" * 80)

        for topic, msg_type in topics:
            print(f"Topic: {topic}")
            print(f"Type:  {msg_type}")

            # Display a trimmed view of the message structure
            structure = self.inspect_message_structure(msg_type)
            if structure.get('fields'):
                print("Fields:")
                for field in structure['fields'][:5]:  # show the first five fields
                    print(f"  - {field['name']}: {field['type']}")
                if len(structure['fields']) > 5:
                    print(f"  ... and {len(structure['fields']) - 5} more fields")
            print("-" * 80)


def main():
    parser = argparse.ArgumentParser(description='ROS2 Topic Inspector for Realtime Mapping')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List all available topics')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Interactive topic selection and config generation')
    parser.add_argument('--output', '-o', type=str,
                       help='Output configuration file path')

    args = parser.parse_args()

    rclpy.init()

    try:
        inspector = TopicInspector()

        if args.list:
            topics = inspector.get_topic_list()
            inspector.print_topic_info(topics)

        elif args.interactive:
            position_topic, sensor_topic = inspector.interactive_selection()
            if position_topic and sensor_topic:
                print(f"\nSelected:")
                print(f"Position: {position_topic[0]} ({position_topic[1]})")
                print(f"Sensor:   {sensor_topic[0]} ({sensor_topic[1]})")

                config = inspector.generate_config_template(position_topic, sensor_topic)
                output_file = args.output or 'config/generated_config.yaml'
                inspector.save_config(config, output_file)

                print(f"\nConfiguration template saved to {output_file}")
                print("Please review and modify the configuration as needed.")

        else:
            parser.print_help()

    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
