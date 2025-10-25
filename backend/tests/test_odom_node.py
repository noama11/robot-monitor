import sys
# sys.path.insert(0, '../src')
from unittest.mock import Mock
import math
from src.nodes.odom_node import OdomNode
from src.data_manager import DataManager
# import sys, os
# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
import pytest
# Helper for approximate float comparison
def approx(val, tol=1e-6):
    return pytest.approx(val, abs=tol)

def test_speed_calculation():

    
    manager = Mock()
    node = OdomNode.__new__(OdomNode)
    node.data_manager = manager
    node.logger = Mock()
    
    msg = Mock()
    msg.twist.twist.linear.x = 3.0
    msg.twist.twist.linear.y = 4.0
    msg.pose.pose.position.x = 0
    msg.pose.pose.position.y = 0
    msg.pose.pose.orientation.x = 0.0 
    msg.pose.pose.orientation.y = 0.0  
    msg.pose.pose.orientation.z = 0
    msg.pose.pose.orientation.w = 1
    msg.header.stamp.sec = 1000
    msg.header.stamp.nanosec = 0
    
    node.odom_callback(msg)
    
    data = manager.update_odom.call_args[0][0]
    assert abs(data['speed'] - 5.0) < 0.01  # sqrt(3^2 + 4^2) = 5
    
    
def test_quaternion_to_yaw_zero_rotation():
    # should be 0 when quaternion represents no rotation
    node = OdomNode.__new__(OdomNode)
    node.logger = Mock()
    yaw = node.quaternion_to_yaw(0, 0, 0, 1) 
    assert yaw == approx(0.0)

def test_quaternion_to_yaw_90_deg_rotation():
    # 90 degree rotation around Z axis
    node = OdomNode.__new__(OdomNode)
    node.logger = Mock()
    yaw = node.quaternion_to_yaw(0, 0, math.sin(math.pi/4), math.cos(math.pi/4)) 
    assert yaw == approx(math.pi/2)
    
def test_quaternion_to_yaw_invalid_values():
    node = OdomNode.__new__(OdomNode)
    node.logger = Mock()
    yaw = node.quaternion_to_yaw(float('nan'), 0, 0, 1) 
    assert yaw == approx(0.0)
    node.logger.warning.assert_called_with('Invalid quaternion values for yaw calculation.')
    
def test_quaternion_to_yaw_180_deg_rotation():
    # 180 degree rotation around Z axis
    node = OdomNode.__new__(OdomNode)
    node.logger = Mock()
    yaw = node.quaternion_to_yaw(0, 0, 1, 0) 
    assert abs(abs(yaw) - math.pi) < 0.01   
def test_quaternion_to_yaw_negative_90_deg_rotation():
    # -90 degree rotation around Z axis
    node = OdomNode.__new__(OdomNode)
    node.logger = Mock()
    yaw = node.quaternion_to_yaw(0, 0, math.sin(-math.pi/4), math.cos(-math.pi/4)) 
    assert yaw == approx(-math.pi/2)    
