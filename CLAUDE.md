# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Development Commands

This is a ROS2 package for realtime 2D sensor mapping. The main package is located in `realtime_mapping/` subdirectory.

### Building
```bash
# Copy package to ROS2 workspace
cp -r realtime_mapping ~/ros2_ws/src/

# Build from ROS2 workspace
cd ~/ros2_ws
colcon build --packages-select realtime_mapping

# Source the built package
source install/setup.bash
```

### Clean rebuild (if needed)
```bash
cd ~/ros2_ws
rm -rf build/realtime_mapping install/realtime_mapping log/realtime_mapping
colcon build --packages-select realtime_mapping
```

### Running the applications

**Note**: Due to entry_points issues in setup.py, executables may not be found via `ros2 run`. Use direct Python execution:

```bash
# Main mapper (direct execution)
cd ~/ros2_ws && source install/setup.bash
python3 src/realtime_mapping/realtime_mapping/realtime_mapper.py --help

# Topic inspector (direct execution)
python3 src/realtime_mapping/realtime_mapping/topic_inspector.py --help

# Interactive topic selection
python3 src/realtime_mapping/realtime_mapping/topic_inspector.py --interactive
```

### Testing dependency installation
```bash
# Install Python dependencies
pip3 install PyYAML matplotlib numpy

# Check ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Architecture Overview

This is a ROS2 package that creates real-time 2D heatmap visualizations from GPS/position data and 1D sensor data.

### Core Components

1. **RealtimeMapper** (`realtime_mapping/realtime_mapper.py`):
   - Main ROS2 node that subscribes to position and sensor topics
   - Creates 1m x 1m cell-based heatmap grids
   - Handles multiple update methods (latest, average, max, min)
   - Provides real-time matplotlib visualization
   - Saves data to CSV and image formats

2. **TopicInspector** (`realtime_mapping/topic_inspector.py`):
   - Utility for discovering available ROS2 topics
   - Generates configuration files automatically
   - Interactive topic selection interface

3. **Configuration System** (`config/*.yaml`):
   - YAML-based configuration for message types and field mappings
   - Supports nested message structures using dot notation (e.g., `pose.pose.position.x`)
   - Supports array access (e.g., `intensities[0]`)

### Message Field Extraction

The system uses a flexible field extraction mechanism:
- **Dot notation**: `pose.pose.position.x` for nested structures
- **Array indexing**: `intensities[0]` for array elements
- **Operator.attrgetter**: Python's attrgetter for dynamic field access

### Coordinate System and Mapping

- **GPS coordinates**: Automatically converted to local coordinate system
- **Cell-based mapping**: Configurable cell size (default 1m x 1m)
- **Grid structure**: numpy arrays for heatmap values and count tracking
- **Update strategies**: latest, average, max, min data aggregation

### Threading Architecture

- **Main thread**: matplotlib visualization and user interaction
- **ROS2 thread**: message callbacks and data processing
- **Thread synchronization**: threading.Lock for shared data access

### Data Pipeline

1. ROS2 messages → Field extraction → Position + sensor value
2. Coordinate transformation → Grid cell mapping
3. Heatmap update (with chosen aggregation method)
4. Real-time visualization update
5. Optional data export (CSV + image)

## Configuration Format

Configuration files specify:
- Input topic names and message types
- Field mapping paths (with nested/array support)
- Sensor value ranges and colormaps
- Map dimensions and cell sizes
- Output format preferences

Example supported message types:
- Position: `sensor_msgs/NavSatFix`, `geometry_msgs/PoseStamped`, `nav_msgs/Odometry`
- Sensors: `std_msgs/Float64`, `sensor_msgs/Range`, `sensor_msgs/LaserScan`

## Known Issues

- **Entry points**: `ros2 run` commands may not work due to setup.py entry_points configuration. Use direct Python execution instead.
- **X11 forwarding**: Required for matplotlib visualization in remote environments
- **Matplotlib warnings**: Multiple matplotlib installations may cause 3D projection warnings