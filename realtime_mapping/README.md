# Realtime 2D Mapping

Realtime 2D Mapping is a ROS 2 package that renders a heatmap from live position and scalar sensor feeds. It supports multiple input pairs (position + sensor) at once, performs the coordinate conversion needed for GPS or local frames, and streams the result to an interactive matplotlib view while optionally exporting CSV snapshots and images.

## Key Features

- **Multi-input fusion** – subscribe to any number of position/sensor topic pairs as defined in a shared YAML configuration.
- **Flexible message bindings** – reference nested fields using dotted/index notation for both position (GPS or Cartesian) and sensor messages.
- **Realtime heatmap visualisation** – configurable cell size, colour map, update rate, and aggregation policy (`latest`, `average`, `max`, `min`).
- **Data export** – save the accumulated map to CSV and PNG on demand or during shutdown.
- **Interactive tooling** – discover topics and bootstrap configuration files with `topic_inspector`, or replay synthetic data with `fake_publisher`.
- **Launch-ready** – comes with a launch file and several example configurations for common input combinations.

## Repository Layout

```
realtime_mapping/
├── BUILD_INSTRUCTIONS.md     # build-from-source guidance (English)
├── config/                   # shared configuration and example presets
├── launch/                   # ROS 2 launch description
├── realtime_mapping/
│   ├── realtime_mapper.py    # main mapping node
│   ├── fake_publisher.py     # synthetic publisher for tests/demos
│   └── topic_inspector.py    # topic discovery and config generator
├── scripts/run_example.sh    # helper script with ready-made scenarios
└── setup.py                  # package manifest
```

## Installation

```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/your-repo/realtime_mapping.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select realtime_mapping

# Source the overlay (bash/zsh)
source install/setup.bash
```

### Runtime Dependencies

- ROS 2 Humble (or newer)
- Python 3.8+
- `numpy`
- `matplotlib`
- `PyYAML`

All pure-Python dependencies are declared in `setup.py`; install them via `pip` if needed when running outside a ROS workspace.

## Quick Start

### 1. Launch the mapper with the shared configuration

```bash
ros2 run realtime_mapping realtime_mapper --config config/message_config.yaml
```

The default configuration maps three inputs simultaneously (`gps_primary`, `gps_secondary`, `local_pose`). The figure window updates automatically as matching data arrives.

### 2. Generate synthetic data (optional demo)

```bash
# In a separate terminal
ros2 run realtime_mapping fake_publisher --config config/message_config.yaml
```

The fake publisher reads the same `inputs` list and publishes circular trajectories with noisy sensor values for every entry where `simulation.enabled` is true.

### 3. Inspect available topics and craft new configs

```bash
# List topics with field previews
ros2 run realtime_mapping topic_inspector --list

# Interactive selection that writes a template config
ros2 run realtime_mapping topic_inspector --interactive --output config/generated_config.yaml
```

You can then merge the generated snippet into `config/message_config.yaml` (or another file) under the `inputs` array.

## Configuration Overview (`config/message_config.yaml`)

```yaml
defaults:
  cell_size: 1.0

sensor_defaults:
  value_range:
    min: 0.0
    max: 100.0

inputs:
  - name: "gps_primary"
    position:
      topic: "/gps/fix"
      message_type: "sensor_msgs/NavSatFix"
      fields:
        latitude: "latitude"
        longitude: "longitude"
    sensor_data:
      topic: "/sensor/data"
      message_type: "std_msgs/Float64"
      fields:
        value: "data"
    simulation:
      enabled: true
      motion:
        radius: 5.0
        rate: 5.0
```

Important sections:

- `sensor_defaults` – shared value range (min/max) used for colormap scaling and fake data synthesis.
- `inputs[]` – each entry defines a position topic and its sensor counterpart. Use dotted/index notation for nested ROS messages, and supply `geo_origin` for GPS feeds or `origin` for Cartesian frames.
- `simulation` – optional hints (`enabled`, `motion`, `value_center`, `value_amplitude`, etc.) consumed by `fake_publisher`.
- `mapping` – heatmap parameters (cell size, map dimensions, origin, aggregation policy, and display tweaks).
- `output` – enable/disable CSV and image export, choose file patterns, DPI, and format.

## Controlling the Visualiser

- Press `s` inside the matplotlib window to dump both CSV and PNG snapshots to the `output/` directory.
- Close the window or press `Ctrl+C` in the terminal to stop the node; shutdown handlers will persist data if enabled.

## Launch File

```bash
ros2 launch realtime_mapping realtime_mapping_launch.py \
  config_file:=config/message_config.yaml \
  interactive:=false
```

Set `interactive:=true` to forward the `--interactive` flag into `realtime_mapper`.

## Example Configurations

The `config/` directory contains presets that you can run directly:

```bash
ros2 run realtime_mapping realtime_mapper --config config/gps_example.yaml
ros2 run realtime_mapping realtime_mapper --config config/robot_pose_example.yaml
ros2 run realtime_mapping realtime_mapper --config config/odom_range_example.yaml
```

Pair them with the fake publisher for repeatable demos:

```bash
ros2 run realtime_mapping fake_publisher --config config/robot_pose_example.yaml
```

## Troubleshooting

| Symptom | Suggested Checks |
| --- | --- |
| Heatmap stays blank | Ensure the chosen position topic publishes before the sensor topic; the mapper needs a position sample to place sensor values. |
| No topics show up in the inspector | Confirm relevant nodes are running (`ros2 topic list`). The inspector filters out `/rosout`, `/tf`, and similar infrastructure topics. |
| GPS data plots in unexpected locations | Verify `geo_origin` latitude/longitude in the configuration. Small errors shift the projected map significantly. |
| Fake publisher is idle | Check that `simulation.enabled` is true for the relevant input and that the topics do not collide with real hardware feeds. |

Enable verbose logging with:

```bash
ros2 run realtime_mapping realtime_mapper --ros-args --log-level debug
```

## Contributing

Issues and pull requests are welcome. Please include:

1. A concise problem description or feature request.
2. Steps to reproduce (if applicable).
3. Proposed changes or supporting data.

## License

This project is released under the MIT License. See the `LICENSE` file for details.
