
import logging
import os


def setup_logging(log_level='INFO'):
    # Setup logging, creates logs directory if needed.

    # Create logs directory if it doesn't exist
    log_dir = 'logs'
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    

    logging.basicConfig(
        level=getattr(logging, log_level),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('logs/robot_monitor.log'),
            logging.StreamHandler()
        ]
    )
    
    logging.info('Logging initialized')