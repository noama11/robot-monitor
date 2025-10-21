#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # Message type for GPS data
from nav_msgs.msg import Odometry  # Message type for robot position and velocity
from sensor_msgs.msg import CompressedImage # for camera images
import json

class RobotDataReader(Node):
    def __init__(self):
        super().__init__('robot_data_reader')
        
        # Store latest data
        self.latest_gps = None
        self.latest_odom = None
        self.latest_speed = 0.0
        
        # Subscribe to GPS topic
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback, # Func to handle incoming data
            10) # Queue size - buffer of incoming messages
        
        # Subscribe to Odometry topic 
        self.odom_sub = self.create_subscription(
            Odometry,
            '/robot_odom',
            self.odom_callback,
            10)
        
        self.get_logger().info('Robot Data Reader is running!')
    
    def gps_callback(self, msg):
        """Process GPS data"""
        # Save the latest GPS info as a dict
        self.latest_gps = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }
        self.get_logger().info(f'GPS: {msg.latitude:.6f}, {msg.longitude:.6f}')
    
    def odom_callback(self, msg):
        """Process Odometry data - speed and orientation"""
        # Calculate speed
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.latest_speed = (vx**2 + vy**2)**0.5  # Total speed
        
        self.latest_odom = {
            'speed': self.latest_speed,
            'position_x': msg.pose.pose.position.x,
            'position_y': msg.pose.pose.position.y,
            'orientation': msg.pose.pose.orientation.z
        }
        
        self.get_logger().info(f'Speed: {self.latest_speed:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = RobotDataReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()