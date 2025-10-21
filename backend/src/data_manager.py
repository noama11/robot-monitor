import logging
from threading import Lock
from collections import deque
from datetime import datetime

class DataManager:
    """
    Central storage for robot sensor data.
    Thread-safe access for GPS, odometry, and camera data
    """
    
    def __init__(self, path_buffer_size=100):
        self.logger = logging.getLogger(__name__)
        
        # Locks for thread safety
        self._gps_lock = Lock()
        self._odom_lock = Lock()
        self._image_lock = Lock()
        
        # Latest data
        self._latest_gps = None
        self._latest_odom = None
        self._latest_image = None
        
        # Path history
        self._gps_path = deque(maxlen=path_buffer_size)
        
        self.logger.info(f'DataManager initialized with path buffer: {path_buffer_size}')
    
    def update_gps(self, gps_data):
        """Store latest GPS data and add to path"""
        with self._gps_lock:
            self._latest_gps = gps_data
            self._gps_path.append({
                'lat': gps_data['latitude'],
                'lng': gps_data['longitude'],
                'timestamp': gps_data.get('timestamp', 0)
            })
        self.logger.debug(f'GPS: {gps_data["latitude"]:.6f}, {gps_data["longitude"]:.6f}')
    
    def update_odom(self, odom_data):
        """Store latest odometry"""
        with self._odom_lock:
            self._latest_odom = odom_data
        self.logger.debug(f'Speed: {odom_data.get("speed", 0):.2f} m/s')
    
    def update_image(self, image_data):
        """Store latest camera image"""
        with self._image_lock:
            self._latest_image = image_data
        self.logger.debug('Image updated')
    
    def get_latest_data(self):
        """
        Get current snapshot of all robot data.
        Returns dict with gps, odom, image, path, and status.
        """
        with self._gps_lock, self._odom_lock, self._image_lock:
            status = 'active' if (self._latest_gps and self._latest_odom) else 'waiting'
            
            return {
                'gps': self._latest_gps,
                'odom': self._latest_odom,
                'image': self._latest_image,
                'path': list(self._gps_path),
                'status': status,
                'timestamp': datetime.now().isoformat()
            }
    
    def clear_path(self):
        with self._gps_lock:
            self._gps_path.clear()