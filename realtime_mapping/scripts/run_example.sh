#!/bin/bash

# Realtime Mapping example launcher

echo "Realtime 2D Mapping - Example"
echo "============================="

# Display available options
echo "1. Run with default configuration"
echo "2. GPS + temperature sensor example"
echo "3. Robot pose + LiDAR example"
echo "4. Odometry + range sensor example"
echo "5. Interactive topic selection"
echo "6. Show available topics"
echo ""

read -p "Choose an option (1-6): " choice

case $choice in
    1)
        echo "Running with default configuration..."
        ros2 run realtime_mapping realtime_mapper --config config/message_config.yaml
        ;;
    2)
        echo "Running GPS + temperature sensor example..."
        ros2 run realtime_mapping realtime_mapper --config config/gps_example.yaml
        ;;
    3)
        echo "Running robot pose + LiDAR example..."
        ros2 run realtime_mapping realtime_mapper --config config/robot_pose_example.yaml
        ;;
    4)
        echo "Running odometry + range sensor example..."
        ros2 run realtime_mapping realtime_mapper --config config/odom_range_example.yaml
        ;;
    5)
        echo "Starting interactive topic selection..."
        ros2 run realtime_mapping topic_inspector --interactive --output config/custom_config.yaml
        if [ -f config/custom_config.yaml ]; then
            echo "Run mapping with the generated configuration? (y/N)"
            read -p "> " run_mapping
            if [ "$run_mapping" = "y" ] || [ "$run_mapping" = "Y" ]; then
                ros2 run realtime_mapping realtime_mapper --config config/custom_config.yaml
            fi
        fi
        ;;
    6)
        echo "Available topics:"
        ros2 run realtime_mapping topic_inspector --list
        ;;
    *)
        echo "Invalid selection."
        ;;
esac
