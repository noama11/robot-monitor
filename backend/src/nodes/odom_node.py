from nav_msgs.msg import Odometry
from .base_node import BaseRobotNode
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class OdomNode(BaseRobotNode):
    """processes robot motion data"""
    
    def __init__(self, config=None, data_manager=None):
        super().__init__('odom_node', config, data_manager)
        
        # Get topic from config
        odom_topic = self.get_topic('odom', '/robot_odom')
        queue_size = self.get_queue_size()
        
        
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            # queue_size
            self.get_sensor_qos()
        )
        
        self.logger.info(f'Subscribed to {odom_topic}')
    
    
    
    def quaternion_to_yaw(self, x, y, z, w):
        """
        convert quaternion to yaw (rotation around Z-axis).
        func returns:
            float: Yaw angle in radians (-π to π)
        """
        # using the formula for yaw from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        
        # validaiton
        if not(math.isfinite(siny_cosp) and math.isfinite(cosy_cosp)):
            self.logger.warning('Invalid quaternion values for yaw calculation.')
            return 0.0
        
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def ros_yaw_to_map_heading(self, yaw_rad):
        """
        Convert ROS yaw  to map heading (North=0, clockwise).
        ROS: 0° = East, 90° = North (counter-clockwise)
        Map: 0° = North, 90° = East (clockwise)   
        returns:
            float: Heading in degrees (0-360), North=0
        """
        # Convert to degrees
        yaw_deg = math.degrees(yaw_rad)
        
        # Convert from ROS (East=0) to Map (North=0, CW)
        # Formula: heading = 90 - yaw
        heading = (90 - yaw_deg + 360) % 360

        return heading
    
    def odom_callback(self, msg):

        """Handle incoming odometry messages."""
        # print(f" ODOM RECEIVED")
        # Calculate speed
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx**2 + vy**2)
        
        # Extract orientation
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # # calc yaw
        # yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        # heading_deg = (math.degrees(yaw) + 360) % 360
        
        
       # calc yaw in ROS frame
        yaw_ros = self.quaternion_to_yaw(qx, qy, qz, qw)
        
        # Convert to map heading (North=0, clockwise)
        heading_deg = self.ros_yaw_to_map_heading(yaw_ros)
        
        # Build data dict
        odom_data = {
            'speed': speed,
            'position_x': msg.pose.pose.position.x,
            'position_y': msg.pose.pose.position.y,
            # 'yaw': yaw, # in radians
            'heading_deg': heading_deg, # in degrees
            'orientation_z': qz,
            'orientation_w': qw,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 #convert time to seconds
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