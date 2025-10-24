from sensor_msgs.msg import NavSatFix
from .base_node import BaseRobotNode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class GPSNode(BaseRobotNode):    
    def __init__(self, config=None, data_manager=None):
        super().__init__('gps_node', config, data_manager)
        
        # Get topic from config
        gps_topic = self.get_topic('gps', '/ublox_gps_node/fix')
        queue_size = self.get_queue_size()
        

        self.subscription = self.create_subscription(
            NavSatFix,
            gps_topic,
            self.gps_callback,
            self.get_sensor_qos()
        )
                
        self.logger.info(f'Subscribed to {gps_topic}')
    

    
    def gps_callback(self, msg):
        # print(f"GPS RECEIVED: {msg.latitude}, {msg.longitude}")

        gps_data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }
        
        if self.data_manager:
            self.data_manager.update_gps(gps_data)


def main(args=None):
    # For Running GPS node only
    import rclpy
    
    rclpy.init(args=args)
    node = GPSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()