#!/usr/bin/env python3
"""Test GPS + Odom together with DataManager."""

import sys
sys.path.insert(0, 'src')

import rclpy
import yaml
from nodes.gps_node import GPSNode
from nodes.odom_node import OdomNode
from data_manager import DataManager
from rclpy.executors import MultiThreadedExecutor
from logger_setup import setup_logging  


def main():
    # Setup logging FIRST
    setup_logging('INFO')  
    
    # Load config
    with open('config/config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    
    # Create shared data manager
    data_manager = DataManager()
    
    # Initialize ROS
    rclpy.init()
    
    # Create both nodes
    print("Starting GPS + Odom Nodes...")
    print("-" * 50)
    
    gps_node = GPSNode(config=config, data_manager=data_manager)
    odom_node = OdomNode(config=config, data_manager=data_manager)
    
    # Use multi-threaded executor to handle both
    executor = MultiThreadedExecutor()
    executor.add_node(gps_node)
    executor.add_node(odom_node)
    
    # Counter
    count = 0
    
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            
            count += 1
            if count % 30 == 0:
                data = data_manager.get_latest_data()
                
                status = "🟢" if data['status'] == 'active' else "🟡"
                
                gps_str = "N/A"
                if data['gps']:
                    gps_str = f"{data['gps']['latitude']:.6f}, {data['gps']['longitude']:.6f}"
                
                speed_str = "N/A"
                if data['odom']:
                    speed_str = f"{data['odom']['speed']:.2f} m/s"
                
                print(f"{status} GPS: {gps_str} | Speed: {speed_str} | Path: {len(data['path'])}")
                    
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        executor.shutdown()
        gps_node.destroy_node()
        odom_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()