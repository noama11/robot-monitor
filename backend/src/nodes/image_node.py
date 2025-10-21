from sensor_msgs.msg import CompressedImage
from .base_node import BaseRobotNode
import base64 # convert to 64 base



class ImageNode(BaseRobotNode):
    
    def __init__(self, config=None, data_manager=None):
        super().__init__('image_node', config, data_manager)
        
        # Check if images are enabled
        self.enabled = self.config.get('data', {}).get('enable_images', False)
        
        if not self.enabled:
            self.logger.info('Image processing disabled in config')
            return
        
        # times per second to process an image
        self.image_count = 0
        self.throttle_rate = self.config.get('data', {}).get('image_throttle', 1)
        
        # Get topic from config
        image_topic = self.get_topic('image', '/neural_depth/left/image_rect_raw/compressed')
        queue_size = self.get_queue_size()
        
        # Subscribe to images
        self.subscription = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            queue_size
        )
        
        self.logger.info(f'Subscribed to {image_topic} (throttle: {self.throttle_rate}Hz)')
    
    def image_callback(self, msg):
        """Handle incoming image messages with throttling."""
        if not self.enabled:
            return
        
        self.image_count += 1
        
        # Throttle: only process every Nth image
        # If throttle_rate=2 and images come at 13Hz, send ~2 images/sec
        skip_factor = max(1, 13 // self.throttle_rate)
        if self.image_count % skip_factor != 0:
            return
        
        # Convert image to base64 for web transmission (so that it can be sent as JSON)
        image_data = {
            'format': msg.format,
            'data': base64.b64encode(msg.data).decode('utf-8'),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }
        
        # Send to data manager
        if self.data_manager:
            self.data_manager.update_image(image_data)


def main(args=None):
    """Run image node standalone."""
    import rclpy
    
    rclpy.init(args=args)
    node = ImageNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()