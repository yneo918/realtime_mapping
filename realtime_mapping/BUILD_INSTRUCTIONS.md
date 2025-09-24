# Build Instructions

## Prerequisites

- ROS 2 Humble (or newer) installed and sourced
- Python 3.8 or newer

## 1. Prepare a ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-repo/realtime_mapping.git
```

> If you already downloaded the repository elsewhere, copy it into `~/ros2_ws/src` instead of cloning again.

## 2. Install dependencies

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

The package depends on `numpy`, `matplotlib`, and `PyYAML`. They are listed in `setup.py`; install them manually if your environment does not resolve them automatically:

```bash
sudo apt update
sudo apt install python3-numpy python3-matplotlib python3-yaml python3-pip
pip3 install --upgrade numpy matplotlib PyYAML
```

## 3. Build

```bash
cd ~/ros2_ws
colcon build --packages-select realtime_mapping
```

Add optional flags as needed:

- `colcon build` – build every package in the workspace.
- `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug` – include debug symbols.

## 4. Source the overlay

```bash
source ~/ros2_ws/install/setup.bash
```

Add the command to your shell profile for convenience:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 5. Sanity checks

```bash
ros2 pkg list | grep realtime_mapping
ros2 run realtime_mapping realtime_mapper --help
ros2 run realtime_mapping topic_inspector --help
```

## 6. Common run commands

```bash
# Mapper with the default configuration
ros2 run realtime_mapping realtime_mapper --config config/message_config.yaml

# Example presets
ros2 run realtime_mapping realtime_mapper --config config/gps_example.yaml
ros2 run realtime_mapping realtime_mapper --config config/robot_pose_example.yaml
ros2 run realtime_mapping realtime_mapper --config config/odom_range_example.yaml

# Topic tools
ros2 run realtime_mapping topic_inspector --list
ros2 run realtime_mapping topic_inspector --interactive --output config/generated_config.yaml

# Synthetic data generator
ros2 run realtime_mapping fake_publisher --config config/message_config.yaml

# Launch description
ros2 launch realtime_mapping realtime_mapping_launch.py

# Example helper script
./scripts/run_example.sh
```

## Troubleshooting

### Build failures

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select realtime_mapping
```

### Missing Python dependencies

```bash
pip3 install --upgrade numpy matplotlib PyYAML
```

Using a virtual environment is also an option:

```bash
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
pip install numpy matplotlib PyYAML
```

### Runtime issues

```bash
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
ros2 daemon stop
ros2 daemon start
ros2 run realtime_mapping realtime_mapper --ros-args --log-level debug
```

## Removing the package

```bash
cd ~/ros2_ws
rm -rf src/realtime_mapping
colcon build
```
