import rclpy
from rclpy.node import Node

class Screen2Node(Node):

    def __init__(self):
        super().__init__('screen2_node')

    def image_edit(self):
        """
        Some kind of Touch Designer integration.
        """
        pass

def main(args=None):
    rclpy.init(args=args)

    screen2_node = Screen2Node()

    rclpy.spin(screen2_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    screen2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


