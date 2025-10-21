import sys
# sys.path.insert(0, '../src')
from unittest.mock import Mock
import math


def test_speed_calculation():
    from src.nodes.odom_node import OdomNode
    
    manager = Mock()
    node = OdomNode.__new__(OdomNode)
    node.data_manager = manager
    node.logger = Mock()
    
    msg = Mock()
    msg.twist.twist.linear.x = 3.0
    msg.twist.twist.linear.y = 4.0
    msg.pose.pose.position.x = 0
    msg.pose.pose.position.y = 0
    msg.pose.pose.orientation.z = 0
    msg.pose.pose.orientation.w = 1
    msg.header.stamp.sec = 1000
    msg.header.stamp.nanosec = 0
    
    node.odom_callback(msg)
    
    data = manager.update_odom.call_args[0][0]
    assert abs(data['speed'] - 5.0) < 0.01  # sqrt(3^2 + 4^2) = 5