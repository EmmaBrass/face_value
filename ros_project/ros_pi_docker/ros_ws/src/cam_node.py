import cv2
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

        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        self.cam_init()

    def cam_init(self):
        """
        Initialize the webcam.
        """
        # Initialize webcam (device 0 is usually the first webcam)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            print("Error: Could not open webcam") # TODO change to logger
            exit(1)

        # Set the resolution to maximum supported by the webcam
        # First try 1080p (1920x1080)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Get actual resolution
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Capturing at resolution: {width}x{height}")

        self.bridge = CvBridge()
        
    def timer_callback(self):
        """
        Publish an image to the cam_image topic.
        """
        print("Hello from the Raspberry Pi!") # TODO change to logger

        # Capture frame
        ret, frame = self.cap.read()

        if not ret:
            print("Error: Could not read frame")
            self.cap.release()
            exit(1)

        # Publish the image
        self.cam_image_publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        self.cap.release() # TODO move this to only run when the node is destroyed
        cv2.destroyAllWindows()

        print("Capture complete!")

# TODO close and open cam connected periodically to avoid freezing up?

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



    


    