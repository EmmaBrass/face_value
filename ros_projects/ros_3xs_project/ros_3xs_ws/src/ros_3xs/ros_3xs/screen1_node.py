import rclpy
from rclpy.node import Node
from config import NUMBER_OF_CAMERAS
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge
import cv2

class Screen1Node(Node):

    def __init__(self):
        super().__init__('screen1_node')
        
        # Create save directory if it doesn't exist
        self.save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'saved_images')
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Initialise image subscribers
        self.cam_image_subscribers = []
        self.bridge = CvBridge()
        for i in range(1, NUMBER_OF_CAMERAS + 1):
            self.cam_image_subscribers.append(self.create_subscription(
                Image,
                f'cam_image_{i}',
                self.cam_image_callback, # TODO need to think about callback groups... this will be called lots of times and they need to get called in parallel. Could have dif image processing nodes for each topic?
                10
            ))

    def cam_image_callback(self, msg):
        """
        Callback function for image subscriber.
        """
        pass

    def image_segmentation(self):
        """
        Perform image segmentation on the received image.
        """
        pass

    def image_processing(self):
        """
        Perform image processing on the received image.
        Choose faces that are good quality, straight on, large enough, not blurred, etc.
        Facial detection to not choose the same person twice.
        """
        pass

    def update_screen(self):
        """
        Update the array of processed images on the screen.
        """
        # TODO use TD for this screen as well? Can Python be used for this?
        
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



    