
import sys
sys.path.insert(0, 'src')

import rclpy
import yaml
from nodes.odom_node import OdomNode
from data_manager import DataManager


def main():
    # Load config
    with open('config/config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    
    # Create data manager
    data_manager = DataManager()
    
    # Initialize ROS
    rclpy.init()
    
    # Create Odom node
    print("Starting Odometry Node...")
    print(f"Topic: {config['ros']['topics']['odom']}")
    print("-" * 50)
    
    odom_node = OdomNode(config=config, data_manager=data_manager)
    
    # Counter for displaying
    count = 0
    
    try:
        while rclpy.ok():
            rclpy.spin_once(odom_node, timeout_sec=0.1)
            
            # Print every 50th update (roughly twice per second at 28Hz)
            count += 1
            if count % 50 == 0:
                data = data_manager.get_latest_data()
                if data['odom']:
                    odom = data['odom']
                    print(f"Speed: {odom['speed']:.2f} m/s | "
                          f"Position: ({odom['position_x']:.2f}, {odom['position_y']:.2f}) | "
                          f"Orientation: {odom['orientation_z']:.3f}")
                    
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        odom_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()