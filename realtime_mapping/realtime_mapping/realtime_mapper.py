#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
import yaml
import csv
import os
from datetime import datetime
import threading
from operator import attrgetter
from functools import partial
import importlib
import argparse
import math
from typing import Dict, Any, Tuple, Optional, List


class RealtimeMapper(Node):
    def __init__(self, config_path: str):
        super().__init__('realtime_mapper')

        # Load the configuration file
        self.config = self.load_config(config_path)

        # Initialize map data structures
        self.initialize_map()

        # Configure ROS 2 subscriptions
        self.setup_subscribers()

        # Initialize visualization and persistence
        self.setup_visualization()
        self.setup_data_storage()

        # Thread control
        self.lock = threading.Lock()
        self.running = True

        self.get_logger().info("Realtime Mapper initialized")

    def load_config(self, config_path: str) -> Dict[str, Any]:
        """Load configuration from disk."""
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f"Config loaded from {config_path}")
            return config
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            raise

    def initialize_map(self):
        """Initialize heatmap data structures."""
        mapping_config = self.config['mapping']
        self.cell_size = mapping_config['cell_size']
        self.map_width = mapping_config['map_size']['width']
        self.map_height = mapping_config['map_size']['height']
        origin_config = mapping_config.get('origin', {'x': 0.0, 'y': 0.0})

        self.origin_x = origin_config.get('x', 0.0)
        self.origin_y = origin_config.get('y', 0.0)
        self.update_method = mapping_config['update_method']

        sensor_defaults = self.config.get('sensor_defaults', {})
        self.sensor_value_range = sensor_defaults.get('value_range', {
            'min': 0.0,
            'max': 100.0
        })

        # Heatmap value and observation counts
        self.heatmap_values = np.zeros((self.map_height, self.map_width))
        self.heatmap_counts = np.zeros((self.map_height, self.map_width))

        # Per-input state tracking
        self.input_states: Dict[str, Dict[str, Any]] = {}
        self.current_positions: Dict[str, Optional[Tuple[float, float]]] = {}
        self.position_history: List[Dict[str, Any]] = []
        self.sensor_data_history: List[Dict[str, Any]] = []

        self.get_logger().info(
            f"Map initialized: {self.map_width}x{self.map_height}, cell_size: {self.cell_size}m"
        )

    def setup_subscribers(self):
        """Create ROS 2 subscriptions for each configured input."""
        try:
            inputs_config = self.config.get('inputs', [])
            if not inputs_config:
                raise ValueError('No inputs defined in configuration')

            mapping_origin = self.config['mapping'].get('origin', {'x': 0.0, 'y': 0.0})

            self.position_subscribers = []
            self.sensor_subscribers = []

            for idx, input_config in enumerate(inputs_config):
                input_name = input_config.get('name', f"input_{idx + 1}")

                pos_config = input_config.get('position', {})
                sensor_config = input_config.get('sensor_data', {})

                if not pos_config or not sensor_config:
                    self.get_logger().warn(f"Input {input_name} is missing position or sensor configuration. Skipping.")
                    continue

                position_topic = pos_config.get('topic', '')
                sensor_topic = sensor_config.get('topic', '')

                pos_msg_type = self.import_message_type(pos_config['message_type'])
                sensor_msg_type = self.import_message_type(sensor_config['message_type'])

                position_fields = pos_config.get('fields', {})
                sensor_fields = sensor_config.get('fields', {})

                uses_navsat = 'latitude' in position_fields and 'longitude' in position_fields

                if uses_navsat:
                    geo_origin_cfg = pos_config.get('geo_origin', {})
                    geo_origin_lon = geo_origin_cfg.get('longitude', mapping_origin.get('x', 0.0))
                    geo_origin_lat = geo_origin_cfg.get('latitude', mapping_origin.get('y', 0.0))
                    reference_lat = geo_origin_lat
                    longitude_scale = 111320.0 * math.cos(math.radians(reference_lat))
                    latitude_scale = 110540.0
                    origin_x = 0.0
                    origin_y = 0.0
                else:
                    geo_origin_lon = None
                    geo_origin_lat = None
                    longitude_scale = None
                    latitude_scale = None
                    origin_config = pos_config.get('origin', {})
                    origin_x = origin_config.get('x', self.origin_x)
                    origin_y = origin_config.get('y', self.origin_y)

                self.input_states[input_name] = {
                    'position_fields': position_fields,
                    'sensor_fields': sensor_fields,
                    'uses_navsat': uses_navsat,
                    'geo_origin_lon': geo_origin_lon,
                    'geo_origin_lat': geo_origin_lat,
                    'longitude_scale': longitude_scale,
                    'latitude_scale': latitude_scale,
                    'origin_x': origin_x,
                    'origin_y': origin_y,
                    'position_topic': position_topic,
                    'sensor_topic': sensor_topic,
                }

                self.current_positions[input_name] = None

                position_callback = partial(self.position_callback, input_name)
                sensor_callback = partial(self.sensor_callback, input_name)

                self.position_subscribers.append(
                    self.create_subscription(
                        pos_msg_type,
                        pos_config['topic'],
                        position_callback,
                        10
                    )
                )

                self.sensor_subscribers.append(
                    self.create_subscription(
                        sensor_msg_type,
                        sensor_config['topic'],
                        sensor_callback,
                        10
                    )
                )

                self.get_logger().info(
                    f"Input '{input_name}' subscribers created for {pos_config['topic']} and {sensor_config['topic']}"
                )

            if not self.input_states:
                self.get_logger().warn('No valid inputs configured for subscriptions')

        except Exception as e:
            self.get_logger().error(f"Failed to setup subscribers: {e}")
            raise

    def import_message_type(self, message_type: str):
        """Resolve a ROS 2 message type string to its class."""
        try:
            parts = message_type.split('/')
            if len(parts) < 2:
                raise ValueError(f"Invalid message type format: {message_type}")

            module_name = f"{parts[0]}.msg"
            class_name = parts[-1]
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except (ImportError, AttributeError, ValueError) as exc:
            self.get_logger().error(f"Failed to import message type {message_type}: {exc}")
            raise

    def get_nested_value(self, obj, field_path: str):
        """Access a nested attribute or sequence value using dot/index notation."""
        try:
            if '[' in field_path and ']' in field_path:
                # Array indexing support
                base_path, array_part = field_path.split('[')
                index = int(array_part.split(']')[0])
                base_value = attrgetter(base_path)(obj) if base_path else obj
                return base_value[index]
            else:
                # Regular dotted path
                return attrgetter(field_path)(obj)
        except Exception as e:
            self.get_logger().warn(f"Failed to get value for {field_path}: {e}")
            return None

    def position_callback(self, input_name: str, msg):
        """Handle incoming position messages for a specific input."""
        try:
            state = self.input_states.get(input_name)
            if state is None:
                return

            pos_fields = state['position_fields']

            if state['uses_navsat']:
                lat_path = pos_fields.get('latitude')
                lon_path = pos_fields.get('longitude')
                if not lat_path or not lon_path:
                    return

                lat = self.get_nested_value(msg, lat_path)
                lon = self.get_nested_value(msg, lon_path)
                if lat is None or lon is None:
                    return

                x = (lon - state['geo_origin_lon']) * state['longitude_scale']
                y = (lat - state['geo_origin_lat']) * state['latitude_scale']
            else:
                x_path = pos_fields.get('x')
                y_path = pos_fields.get('y')
                if not x_path or not y_path:
                    return

                x = self.get_nested_value(msg, x_path)
                y = self.get_nested_value(msg, y_path)

            if x is not None and y is not None:
                with self.lock:
                    self.current_positions[input_name] = (x, y)
                    self.position_history.append({
                        'input': input_name,
                        'timestamp': self.get_clock().now().to_msg(),
                        'x': x,
                        'y': y
                    })

        except Exception as e:
            self.get_logger().warn(f"Position callback error ({input_name}): {e}")

    def sensor_callback(self, input_name: str, msg):
        """Handle incoming sensor messages for a specific input."""
        try:
            state = self.input_states.get(input_name)
            if state is None:
                return

            sensor_fields = state['sensor_fields']
            value_path = sensor_fields.get('value')
            if not value_path:
                return

            sensor_value = self.get_nested_value(msg, value_path)

            if sensor_value is not None:
                with self.lock:
                    position = self.current_positions.get(input_name)
                    if position is None:
                        return
                    x, y = position
                    self.update_heatmap(x, y, sensor_value)

                    # Store raw data with topic names for time-series export
                    pos_topic = state['position_topic']
                    sensor_topic = state['sensor_topic']

                    self.sensor_data_history.append({
                        'input': input_name,
                        'timestamp': self.get_clock().now().to_msg(),
                        'position_topic': pos_topic,
                        'sensor_topic': sensor_topic,
                        'x': x,
                        'y': y,
                        'value': sensor_value
                    })

        except Exception as e:
            self.get_logger().warn(f"Sensor callback error ({input_name}): {e}")

    def update_heatmap(self, x: float, y: float, value: float):
        """Update the heatmap with a new observation."""
        # Convert world coordinates into cell indices
        cell_x = int((x - self.origin_x) / self.cell_size + self.map_width // 2)
        cell_y = int((y - self.origin_y) / self.cell_size + self.map_height // 2)

        # Bounds check
        if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
            if self.update_method == "latest":
                self.heatmap_values[cell_y, cell_x] = value
                self.heatmap_counts[cell_y, cell_x] += 1
            elif self.update_method == "average":
                old_count = self.heatmap_counts[cell_y, cell_x]
                new_count = old_count + 1
                old_avg = self.heatmap_values[cell_y, cell_x]
                new_avg = value if old_count == 0 else (old_avg * old_count + value) / new_count
                self.heatmap_values[cell_y, cell_x] = new_avg
                self.heatmap_counts[cell_y, cell_x] = new_count
            elif self.update_method == "max":
                if self.heatmap_counts[cell_y, cell_x] == 0:
                    self.heatmap_values[cell_y, cell_x] = value
                else:
                    self.heatmap_values[cell_y, cell_x] = max(self.heatmap_values[cell_y, cell_x], value)
                self.heatmap_counts[cell_y, cell_x] += 1
            elif self.update_method == "min":
                if self.heatmap_counts[cell_y, cell_x] == 0:
                    self.heatmap_values[cell_y, cell_x] = value
                else:
                    self.heatmap_values[cell_y, cell_x] = min(self.heatmap_values[cell_y, cell_x], value)
                self.heatmap_counts[cell_y, cell_x] += 1

    def setup_visualization(self):
        """Configure matplotlib visualization."""
        display_config = self.config['mapping'].get('display', {})

        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title('Realtime 2D Sensor Mapping')

        # Initial heatmap state
        sensor_range = self.sensor_value_range
        base_cmap = plt.get_cmap(display_config['colormap'])
        if hasattr(base_cmap, 'copy'):
            cmap = base_cmap.copy()
        else:
            cmap = ListedColormap(
                base_cmap(np.linspace(0, 1, base_cmap.N)),
                name=f"{base_cmap.name}_masked"
            )
        cmap.set_bad(color=self.ax.get_facecolor())

        initial_image = np.ma.array(
            self.heatmap_values,
            mask=self.heatmap_counts == 0
        )

        self.im = self.ax.imshow(
            initial_image,
            cmap=cmap,
            vmin=sensor_range['min'],
            vmax=sensor_range['max'],
            origin='lower',
            extent=[
                self.origin_x - self.map_width * self.cell_size / 2,
                self.origin_x + self.map_width * self.cell_size / 2,
                self.origin_y - self.map_height * self.cell_size / 2,
                self.origin_y + self.map_height * self.cell_size / 2
            ]
        )

        show_colorbar = display_config.get('show_colorbar', True)
        if show_colorbar:
            plt.colorbar(self.im, ax=self.ax)

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')

        self.grid_lines = []

        # Marker for current positions
        self.position_marker, = self.ax.plot([], [], 'ro', markersize=8, label='Current Positions')
        self.ax.legend()

        # Animation timing configuration
        self.update_rate = display_config.get('update_rate', 10.0)

        plt.tight_layout()

    def setup_data_storage(self):
        """Configure data persistence resources."""
        self.output_config = self.config['output']

        # Ensure the output directory exists
        os.makedirs('output', exist_ok=True)

    def update_visualization(self, frame):
        """Refresh the animation frame."""
        with self.lock:
            if self.heatmap_values is not None:
                masked_values = np.ma.array(
                    self.heatmap_values,
                    mask=self.heatmap_counts == 0
                )
                self.im.set_array(masked_values)

                positions = [pos for pos in self.current_positions.values() if pos is not None]
                if positions:
                    xs, ys = zip(*positions)
                else:
                    xs, ys = [], []
                self.position_marker.set_data(xs, ys)

        return [self.im, self.position_marker]

    def save_csv_data(self):
        """Persist heatmap data to CSV."""
        if not self.output_config['csv']['enabled']:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = self.output_config['csv']['filename_pattern'].format(timestamp=timestamp)
        filepath = os.path.join('output', filename)

        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Metadata header
            if self.output_config['csv']['include_metadata']:
                writer.writerow(['# Realtime Mapping Data'])
                writer.writerow(['# Timestamp:', timestamp])
                writer.writerow(['# Cell Size (m):', self.cell_size])
                writer.writerow(['# Map Size:', f"{self.map_width}x{self.map_height}"])
                writer.writerow([])

            writer.writerow(['Cell_X', 'Cell_Y', 'World_X', 'World_Y', 'Value', 'Count'])

            # Heatmap data rows
            for y in range(self.map_height):
                for x in range(self.map_width):
                    if self.heatmap_counts[y, x] > 0:
                        world_x = (x - self.map_width // 2) * self.cell_size + self.origin_x
                        world_y = (y - self.map_height // 2) * self.cell_size + self.origin_y
                        writer.writerow([
                            x, y, world_x, world_y,
                            self.heatmap_values[y, x],
                            self.heatmap_counts[y, x]
                        ])

        self.get_logger().info(f"CSV data saved to {filepath}")

    def save_raw_timeseries_data(self):
        """Persist all received sensor data as time-series CSV."""
        if not self.output_config['csv']['enabled']:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"raw_timeseries_{timestamp}.csv"
        filepath = os.path.join('output', filename)

        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Header
            writer.writerow(['Timestamp', 'Position_Topic', 'Sensor_Topic', 'Pos_X', 'Pos_Y', 'Scalar'])

            # Time-series data rows
            for record in self.sensor_data_history:
                # Convert ROS timestamp to seconds
                ts = record['timestamp']
                timestamp_sec = ts.sec + ts.nanosec * 1e-9

                writer.writerow([
                    timestamp_sec,
                    record['position_topic'],
                    record['sensor_topic'],
                    record['x'],
                    record['y'],
                    record['value']
                ])

        self.get_logger().info(f"Raw time-series data saved to {filepath}")

    def save_image(self):
        """Persist the heatmap as an image."""
        if not self.output_config['image']['enabled']:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = self.output_config['image']['filename_pattern'].format(timestamp=timestamp)
        filepath = os.path.join('output', filename)

        self.fig.savefig(
            filepath,
            dpi=self.output_config['image']['dpi'],
            format=self.output_config['image']['format'],
            bbox_inches='tight'
        )

        self.get_logger().info(f"Image saved to {filepath}")

    def start_visualization(self):
        """Launch the interactive visualization window."""
        ani = animation.FuncAnimation(
            self.fig, self.update_visualization,
            interval=1000/self.update_rate,
            blit=True
        )

        # Key binding for quick exports
        def on_key(event):
            if event.key == 's':
                self.save_csv_data()
                self.save_raw_timeseries_data()
                self.save_image()

        self.fig.canvas.mpl_connect('key_press_event', on_key)

        plt.show()

    def shutdown(self):
        """Perform shutdown bookkeeping."""
        self.running = False
        self.save_csv_data()
        self.save_raw_timeseries_data()
        self.save_image()
        self.get_logger().info("Realtime Mapper shutdown")


def get_available_topics():
    """Return a filtered list of available ROS 2 topics."""
    import subprocess

    try:
        result = subprocess.run(['ros2', 'topic', 'list', '-t'],
                              capture_output=True, text=True)
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
        print(f"Failed to get topics: {e}")
        return []


def interactive_topic_selection():
    """Prompt the user to select position and sensor topics interactively."""
    print("Available topics:")
    topics = get_available_topics()

    if not topics:
        print("No topics found. Make sure ROS2 nodes are running.")
        return None, None

    for i, (topic, msg_type) in enumerate(topics):
        print(f"{i+1}: {topic} ({msg_type})")

    while True:
        try:
            choice = input("\nSelect position topic (number): ")
            pos_idx = int(choice) - 1
            if 0 <= pos_idx < len(topics):
                position_topic = topics[pos_idx]
                break
        except ValueError:
            pass
        print("Invalid selection. Please try again.")

    while True:
        try:
            choice = input("Select sensor topic (number): ")
            sensor_idx = int(choice) - 1
            if 0 <= sensor_idx < len(topics):
                sensor_topic = topics[sensor_idx]
                break
        except ValueError:
            pass
        print("Invalid selection. Please try again.")

    return position_topic, sensor_topic


def main():
    parser = argparse.ArgumentParser(description='Realtime 2D Mapping with ROS2')
    parser.add_argument('--config', '-c',
                       default='config/message_config.yaml',
                       help='Configuration file path')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Interactive topic selection')

    args = parser.parse_args()

    rclpy.init()

    try:
        if args.interactive:
            pos_topic, sensor_topic = interactive_topic_selection()
            if pos_topic and sensor_topic:
                print(f"Selected: Position={pos_topic[0]}, Sensor={sensor_topic[0]}")
                # Update configuration dynamically (implementation omitted)

        mapper = RealtimeMapper(args.config)

        # Run ROS 2 spinning loop on a background thread
        ros_thread = threading.Thread(target=lambda: rclpy.spin(mapper))
        ros_thread.daemon = True
        ros_thread.start()

        # Drive the visualization on the main thread
        mapper.start_visualization()

    except KeyboardInterrupt:
        pass
    finally:
        if 'mapper' in locals():
            mapper.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
