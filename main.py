import cv2
from robot_control import robot_operation
from grpc_image_streaming.server.image_receiver import ImageStreamerServicer
from logger import image_logger as logger

@robot_operation(mode='simulation')
def example_movement(robot):
    """
    Example robot movement function.
    
    :param robot: RobotController instance
    """
    # Perform relative movement
    robot.controller.move_base_relative(
        20.0, 20.0, 20.0,  # x, y, z movement
        0.0, 0.0, 0.0,     # rotational movement
        0.0, 0.0, 0.0,     # tool coordinates
        10.0,              # speed
        wait_move_finished=True,
        move_finished_timeout=1000
    )

def receive_image():
    """
    Example of receiving an image from the image receiver.
    """

    # Example of getting the latest image
    # Get the most recent image from any source
    latest_image = ImageStreamerServicer.get_latest_image()
    latest_image_path = ImageStreamerServicer.get_latest_image_path()
    
    if latest_image is not None:
        logger.info(f'Latest image path: {latest_image_path}')
        logger.info(f'Latest image shape: {latest_image.shape}')
        
        # Optional: display the image (requires a GUI)
        cv2.imshow('Latest Image', latest_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # List all sources that have sent images
    sources = ImageStreamerServicer.list_sources()
    logger.info('Image sources:', sources)

    # Example of getting image from a specific source
    if sources:
        specific_source = sources[0]  # Get first source
        specific_image = ImageStreamerServicer.get_latest_image(specific_source)
        specific_image_path = ImageStreamerServicer.get_latest_image_path(specific_source)
        
        if specific_image is not None:
            logger.info(f'Image from source {specific_source}')
            logger.info(f'Image path: {specific_image_path}')
            logger.info(f'Image shape: {specific_image.shape}')

if __name__ == '__main__':
    # Demonstrate usage
    example_movement()
    #receive_image()
