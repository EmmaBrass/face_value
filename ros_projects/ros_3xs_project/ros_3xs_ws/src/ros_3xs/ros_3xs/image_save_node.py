import rclpy
from rclpy.node import Node
from . import config
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge
import cv2
import sys
sys.stdout.reconfigure(line_buffering=True)


class ImageSaveNode(Node):

    def __init__(self):
        super().__init__('image_save_node')

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Create save directory if it doesn't exist
        self.save_dir = config.IMAGE_FILE
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Initialise image subscribers
        # self.cam_image_subscribers = []
        self.bridge = CvBridge()

        self.cam_image_subscriber = self.create_subscription(
            Image,
            'cam_image_1',
            self.cam_image_callback, 
            10
        )

        self.get_logger().info("ImageSaveNode has started")

        # for i in range(1, config.NUMBER_OF_CAMERAS + 1):
        #     self.cam_image_subscribers.append(self.create_subscription(
        #         Image,
        #         f'cam_image_{i}',
        #         self.cam_image_callback, # TODO need to think about callback groups... this will be called lots of times and they need to get called in parallel. Could have dif image processing nodes for each topic?
        #         10
        #     ))

    def cam_image_callback(self, msg):
        """
        Callback function for image subscriber.
        Saves the latest image from each camera to a folder.
        """
        self.get_logger().info("In image callback")
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Get camera number from topic name
            cam_num = 1
            
            now = self.get_clock().now().to_msg()
            timestamp = now.sec
            
            filename = f'cam_{cam_num}_{timestamp}.jpg'
            filepath = os.path.join(self.save_dir, filename)
            
            # Save the image
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f'Saved image from camera {cam_num} to {filepath}')
        except Exception as e:
            self.get_logger().error(f'Error saving image: {str(e)}')

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
        pass

def main(args=None):
    rclpy.init(args=args)

    image_save_node = ImageSaveNode()

    rclpy.spin(image_save_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_save_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    