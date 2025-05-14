import grpc
import cv2
import numpy as np
import os
from concurrent import futures
import sys
sys.path.append('/Users/Emma/Documents/Documents_MacBook_Pro/pen_plotter/RoL 2025/rol_2025/face_value/grpc_image_streaming/proto')

import image_stream_pb2
import image_stream_pb2_grpc

class ImageStreamerServicer(image_stream_pb2_grpc.ImageStreamerServicer):
    def __init__(self, output_dir='/tmp/image_stream'):
        os.makedirs(output_dir, exist_ok=True)
        self.output_dir = output_dir
        self.frame_count = 0

    def StreamFrames(self, request_iterator, context):
        for image_frame in request_iterator:
            # Convert compressed JPEG to OpenCV image
            nparr = np.frombuffer(image_frame.image_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            # Save frame with timestamp
            filename = os.path.join(self.output_dir, f'frame_{image_frame.timestamp}_{self.frame_count}.jpg')
            cv2.imwrite(filename, frame)
            self.frame_count += 1

            # Yield a response
            yield image_stream_pb2.ServerResponse(
                status=image_stream_pb2.ServerResponse.Status.OK,
                message=f'Received frame {self.frame_count}'
            )

def serve(port=50051):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    image_stream_pb2_grpc.add_ImageStreamerServicer_to_server(
        ImageStreamerServicer(), server)
    server.add_insecure_port(f'[::]:{port}')
    print(f'Starting gRPC server on port {port}')
    server.start()
    server.wait_for_termination()

if __name__ == '__main__':
    serve()
