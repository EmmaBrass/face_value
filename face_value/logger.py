import logging
import os
from datetime import datetime

def setup_logger(name, log_directory=None):
    """
    Create a standardized logger with console and file logging.
    
    :param name: Name of the logger
    :param log_directory: Optional directory to store log files. 
                          If None, uses a 'logs' directory in the project root.
    :return: Configured logger
    """
    # Determine log directory
    if log_directory is None:
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        log_directory = os.path.join(project_root, 'logs')
    
    # Create log directory if it doesn't exist
    os.makedirs(log_directory, exist_ok=True)
    
    # Create logger
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    
    # Clear any existing handlers to prevent duplicate logging
    logger.handlers.clear()
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Console Handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # File Handler with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_directory, f'{name}_{timestamp}.log')
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    
    return logger

# Global loggers for common components
robot_logger = setup_logger('robot')
image_logger = setup_logger('image')
system_logger = setup_logger('system')

# Example usage:
# from face_value.logger import robot_logger, image_logger, system_logger
# robot_logger.info('Robot movement started')
# image_logger.error('Failed to process image')
# system_logger.warning('Potential system issue detected')
