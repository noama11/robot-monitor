import rclpy
from rclpy.node import Node
import logging
import yaml
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class BaseRobotNode(Node):
    """
    Base class for all robot sensor nodes.
    Provides common setup: logging, config loading, data manager connection.
    """
    
    def __init__(self, node_name, config=None, data_manager=None):
        super().__init__(node_name)
        
        self.logger = logging.getLogger(node_name)
        self.data_manager = data_manager
        self.config = config or {}
        
        self.logger.info(f'{node_name} initialized')
    
    
    def get_topic(self, key, default):
        # Get topic name from config
        
        topics = self.config.get('ros', {}).get('topics', {})
        return topics.get(key, default)
    
    def get_queue_size(self):
        return self.config.get('ros', {}).get('queue_size', 10)
    
    def get_sensor_qos(self):
        """QoS profile for sensor data from bag files."""
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=self.get_queue_size()
        )