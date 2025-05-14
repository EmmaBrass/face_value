from face_value.logger import image_logger as logger
import os
import time
import grpc
import cv2
import numpy as np
import sys
sys.path.append('/Users/Emma/Documents/Documents_MacBook_Pro/pen_plotter/RoL 2025/rol_2025/face_value/face_value/grpc_image_streaming/proto')

import image_stream_pb2
import image_stream_pb2_grpc

class TestImageSender:
    def __init__(self, test_images_dir, server_address='localhost:50051'):
        self.test_images_dir = test_images_dir
        self.server_address = server_address

    def get_test_images(self):
        """Yield test images from the directory."""
        image_files = [f for f in os.listdir(self.test_images_dir) if f.endswith(('.jpg', '.png', '.jpeg'))]
        while True:
            for image_file in image_files:
                yield os.path.join(self.test_images_dir, image_file)
                time.sleep(2)  # Send an image every 2 seconds

    def send_images(self):
        """Send test images to the gRPC server."""
        try:
            channel = grpc.insecure_channel(self.server_address)
            stub = image_stream_pb2_grpc.ImageStreamerStub(channel)

            def image_generator():
                for image_path in self.get_test_images():
                    # Read image
                    frame = cv2.imread(image_path)
                    
                    # Compress image to JPEG
                    _, buffer = cv2.imencode('.jpg', frame)
                    
                    # Create image frame
                    image_frame = image_stream_pb2.ImageFrame(
                        image_data=buffer.tobytes(),
                        timestamp=int(time.time())
                    )
                    yield image_frame

            # Stream frames
            responses = stub.StreamFrames(image_generator())
            
            for response in responses:
                logger.info(f"Server response: {response.message}")

        except grpc.RpcError as e:
            logger.error(f"RPC Error: {e}")
        except Exception as e:
            logger.error(f"Error sending images: {e}")

def main():
    test_images_dir = '/Users/Emma/Documents/Documents_MacBook_Pro/pen_plotter/RoL 2025/rol_2025/face_value/face_value/grpc_image_streaming/test_images'
    sender = TestImageSender(test_images_dir)
    sender.send_images()

if __name__ == '__main__':
    main()
