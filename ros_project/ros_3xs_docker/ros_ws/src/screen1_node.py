import rclpy
from rclpy.node import Node

class Screen1Node(Node):

    def __init__(self):
        super().__init__('screen1_node')

    def image_segmentation(self):
        """
        Perform image segmentation on the received image.
        """
        pass

    def image_processing(self):
        """
        Perform image processing on the received image.
        Choose faces that are good quality, straight on, large enough, not blurred, etc.
        """
        pass

def main(args=None):
    rclpy.init(args=args)

    screen1_node = Screen1Node()

    rclpy.spin(screen1_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    screen1_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    