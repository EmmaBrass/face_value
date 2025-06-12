
import rclpy
from rclpy.node import Node

class ConductorNode(Node):

    def __init__(self):
        super().__init__('conductor_node')

    def start_up(self):
        """
        Whatever processes happen at startup.
        """
        pass


        

def main(args=None):
    rclpy.init(args=args)

    conductor_node = ConductorNode()

    rclpy.spin(conductor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    conductor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


