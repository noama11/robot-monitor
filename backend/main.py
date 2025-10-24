#!/usr/bin/env python3
import sys, os
sys.path.insert(0, 'src')

import yaml
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
import os

from logger_setup import setup_logging
from data_manager import DataManager
from nodes.gps_node import GPSNode
from nodes.odom_node import OdomNode
from nodes.image_node import ImageNode
from websocket_server import run_server

# Main entry point for robot monitor backend


def load_config():
    """Load configuration from YAML file."""
    config_path = 'config/config.yaml'
    
    if not os.path.exists(config_path):
        print(f"ERROR: Config file not found: {config_path}")
        print("Please ensure config/config.yaml exists.")
        sys.exit(1)
    
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"ERROR: Failed to load config: {e}")
        sys.exit(1)



def main():
    # Setup logging
    try:
        setup_logging('INFO')
    except Exception as e:
        print(f"ERROR: Failed to setup logging: {e}")
        sys.exit(1)
    
    # Load config
    config = load_config()
    print("Robot Monitor Backend")
    
    # Create shared data manager
    data_manager = DataManager(
        path_buffer_size=config.get('data', {}).get('path_buffer_size', 100)
    )
    
    # Initialize ROS
    try:
        rclpy.init()
    except Exception as e:
        print(f"ERROR: Failed to initialize ROS: {e}")
        print("Make sure ROS 2 is installed and sourced.")
        sys.exit(1)
    
    # Create ROS nodes
    try:
        gps_node = GPSNode(config=config, data_manager=data_manager)
        odom_node = OdomNode(config=config, data_manager=data_manager)
        image_node = ImageNode(config=config, data_manager=data_manager)
    except Exception as e:
        print(f"ERROR: Failed to create nodes: {e}")
        rclpy.shutdown()
        sys.exit(1)
    
    # Setup ROS executor
    executor = MultiThreadedExecutor()
    executor.add_node(gps_node)
    executor.add_node(odom_node)
    if config.get('data', {}).get('enable_images', False):
        executor.add_node(image_node)
    
    # Start WebSocket server in separate thread
    try:
        ws_thread = threading.Thread(
            target=run_server,
            args=(data_manager, config),
            daemon=True
        )
        ws_thread.start()
    except Exception as e:
        print(f"ERROR: Failed to start WebSocket server: {e}")
        executor.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    
    print("All systems running")
    
    
    import time
    print("\n=== Checking ROS Node Status ===")
    print(f"GPS Node: {gps_node.get_name()}")
    print(f"Odom Node: {odom_node.get_name()}")
    print(f"Image Node: {image_node.get_name()}")
    
    # Run ROS nodes
    try:
        executor.spin()  # Listen to topics 
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"ERROR during execution: {e}")
    finally:
        executor.shutdown()
        gps_node.destroy_node()
        odom_node.destroy_node()
        image_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()