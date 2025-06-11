import rclpy
from rclpy.node import Node

class Screen3Node(Node):

    def __init__(self):
        super().__init__('screen3_node')

    def deposit_image_in_folder(self):
        """
        Deposit the image in a folder, so that it can be accessed by Unity workflow.
        """
        pass


def main(args=None):
    rclpy.init(args=args)

    screen3_node = Screen3Node()

    rclpy.spin(screen3_node)

    screen3_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
