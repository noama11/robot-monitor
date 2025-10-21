from nav_msgs.msg import Odometry
from .base_node import BaseRobotNode
import math


class OdomNode(BaseRobotNode):
    """processes robot motion data"""
    
    def __init__(self, config=None, data_manager=None):
        super().__init__('odom_node', config, data_manager)
        
        # Get topic from config
        odom_topic = self.get_topic('odom', '/robot_odom')
        queue_size = self.get_queue_size()
        
        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            queue_size
        )
        
        self.logger.info(f'Subscribed to {odom_topic}')
    
    def odom_callback(self, msg):
        """Handle incoming odometry messages."""
        # Calculate speed
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)
        
        # Extract orientation
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Build data dict
        odom_data = {
            'speed': speed,
            'position_x': msg.pose.pose.position.x,
            'position_y': msg.pose.pose.position.y,
            'orientation_z': qz,
            'orientation_w': qw,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }
        
        if self.data_manager:
            self.data_manager.update_odom(odom_data)


def main(args=None):
    """Run odometry node standalone."""
    import rclpy
    
    rclpy.init(args=args)
    node = OdomNode()
    
    try:
        rclpy.spin(node) #listening in a loop
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()