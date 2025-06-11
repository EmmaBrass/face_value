import os
from functools import wraps
from typing import Optional, Callable

from cri_lib import CRIController

# Configuration for robot connection
ROBOT_CONFIG = {
    'simulation': {
        'ip': '127.0.0.1',
        'port': 3921
    },
    'real': {
        'ip': '192.168.3.11',
        'port': 3920
    }
}

class RobotController:
    def __init__(self, mode: str = 'simulation'):
        """
        Initialize robot controller with simulation or real-robot mode.
        
        :param mode: 'simulation' or 'real'
        """
        if mode not in ['simulation', 'real']:
            raise ValueError("Mode must be 'simulation' or 'real'")
        
        self.mode = mode
        self.controller = CRIController()
        self._config = ROBOT_CONFIG[mode]
    
    def connect(self, timeout: int = 10) -> bool:
        """
        Connect to robot based on current mode.
        
        :param timeout: Connection timeout in seconds
        :return: Connection success status
        """
        try:
            connection_status = self.controller.connect(
                self._config['ip'], 
                self._config.get('port')
            )
            
            if not connection_status:
                print(f"Unable to connect to {self.mode} robot at {self._config['ip']}:{self._config.get('port', 'default port')}")
                return False
            
            print(f"Connected to {self.mode} robot successfully")
            return True
        
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def prepare_robot(self) -> bool:
        """
        Prepare robot for operation: acquire control, enable, and wait for readiness.
        
        :return: Preparation success status
        """
        try:
            # Acquire active control
            if not self.controller.set_active_control(True):
                print("Failed to acquire active control")
                return False
            
            # Enable motors
            if not self.controller.enable():
                print("Failed to enable motors")
                return False
            
            # Wait for kinematics to be ready
            if not self.controller.wait_for_kinematics_ready(10):
                print("Kinematics not ready within timeout")
                return False
            
            print("Robot is prepared and ready")
            return True
        
        except Exception as e:
            print(f"Robot preparation error: {e}")
            return False
    
    def cleanup(self):
        """
        Disable motors and close connection.
        """
        try:
            self.controller.disable()
            self.controller.close()
            print("Robot connection closed")
        except Exception as e:
            print(f"Error during robot cleanup: {e}")
    
    def __enter__(self):
        """Context manager entry point."""
        self.connect()
        self.prepare_robot()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit point."""
        self.cleanup()

def robot_operation(func: Optional[Callable] = None, *, mode: str = 'simulation'):
    """
    Decorator to wrap robot operations with automatic connection and cleanup.
    
    :param func: Function to decorate
    :param mode: 'simulation' or 'real'
    :return: Decorated function
    """
    def decorator(operation):
        @wraps(operation)
        def wrapper(*args, **kwargs):
            with RobotController(mode=mode) as robot:
                return operation(robot, *args, **kwargs)
        return wrapper
    
    # Allow decorator to be used with or without parentheses
    return decorator(func) if func else decorator