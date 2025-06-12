import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CamNode(Node):

    def __init__(self):
        super().__init__('cam_node')

        # Get cam_id from launch file
        self.declare_parameter('cam_id', 0)
        self.cam_id = self.get_parameter('cam_id').get_parameter_value().integer_value

        # Initialise image publisher
        self.cam_image_publisher = self.create_publisher(Image, f'cam_image_{self.cam_id}', 10)
        timer_period = 5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cam_init()

    def cam_init(self):
        """
        Initialise the webcam.
        """
        import cv2
        self.cap = cv2.VideoCapture(1)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open webcam')
            exit(1)

        # Set the resolution to maximum supported by the webcam
        # First try 1080p (1920x1080)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Get actual resolution
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Capturing at resolution: {width}x{height}')

    def timer_callback(self):
        """
        Publish an image to the cam_image topic.
        """
        self.get_logger().info("In timer callback")

        # Capture frame and publish
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.cam_image_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    cam_node = CamNode()

    rclpy.spin(cam_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()