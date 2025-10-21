"""Test GPS node with real bag data."""

import sys
sys.path.insert(0, 'src')

import rclpy
import yaml
from nodes.gps_node import GPSNode
from data_manager import DataManager


def main():
    # Load config
    with open('config/config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    
    # Create data manager
    data_manager = DataManager()
    
    # Initialize ROS
    rclpy.init()
    
    # Create GPS node
    print("Starting GPS Node...")
    print(f"Topic: {config['ros']['topics']['gps']}")
    print("-" * 50)
    
    gps_node = GPSNode(config=config, data_manager=data_manager)
    
    # Counter for displaying
    count = 0
    
    try:
        while rclpy.ok():
            rclpy.spin_once(gps_node, timeout_sec=0.1)
            
            # Print every 10th update (roughly once per second at 4Hz)
            count += 1
            if count % 10 == 0:
                data = data_manager.get_latest_data()
                if data['gps']:
                    gps = data['gps']
                    print(f"GPS: lat={gps['latitude']:.6f}, "
                          f"lon={gps['longitude']:.6f}, "
                          f"alt={gps['altitude']:.2f}m | "
                          f"Path points: {len(data['path'])}")
                    
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        gps_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()