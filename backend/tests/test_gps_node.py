import sys
# sys.path.insert(0, '../src')
from unittest.mock import Mock


def test_gps_callback():
    from src.nodes.gps_node import GPSNode
    
    manager = Mock()
    node = GPSNode.__new__(GPSNode)
    node.data_manager = manager
    node.logger = Mock()
    
    msg = Mock()
    msg.latitude = 32.0853
    msg.longitude = 34.7818
    msg.altitude = 50.0
    msg.header.stamp.sec = 1000
    msg.header.stamp.nanosec = 0
    
    node.gps_callback(msg)
    
    assert manager.update_gps.called
    data = manager.update_gps.call_args[0][0]
    assert data['latitude'] == 32.0853