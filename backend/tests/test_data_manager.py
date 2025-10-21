import sys
# sys.path.insert(0, '../src')
from src.data_manager import DataManager


def test_stores_gps():
    manager = DataManager()
    manager.update_gps({'latitude': 32.0, 'longitude': 34.0, 'altitude': 50.0, 'timestamp': 0})
    
    data = manager.get_latest_data()
    assert data['gps']['latitude'] == 32.0


def test_stores_odom():
    manager = DataManager()
    manager.update_odom({'speed': 2.5, 'position_x': 10.0, 'timestamp': 0})
    
    data = manager.get_latest_data()
    assert data['odom']['speed'] == 2.5


def test_path_grows():
    manager = DataManager()
    
    for i in range(5):
        manager.update_gps({'latitude': 32.0, 'longitude': 34.0, 'altitude': 50.0, 'timestamp': i})
    
    assert len(manager.get_latest_data()['path']) == 5