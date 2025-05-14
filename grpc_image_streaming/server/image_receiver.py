import grpc
import cv2
import numpy as np
import os
from concurrent import futures
import sys
import time
import threading
from datetime import datetime, timedelta
sys.path.append(os.path.join(os.path.dirname(__file__), 'proto'))

# Import custom logger
from logger import image_logger as logger

from ..proto import image_stream_pb2
from ..proto import image_stream_pb2_grpc

class ImageStreamerServicer(image_stream_pb2_grpc.ImageStreamerServicer):
    # Dictionary to store latest images from different sources
    _latest_images = {}
    
    # Dictionary to track device connection status
    _device_connections = {}
    
    # Maximum time to consider a device disconnected
    _max_disconnection_time = timedelta(minutes=5)

    def __init__(self, output_dir='/tmp/image_stream'):
        os.makedirs(output_dir, exist_ok=True)
        self.output_dir = output_dir
        self.frame_count = 0

    def StreamFrames(self, request_iterator, context):
        # Unique device identifier (you might want to pass this from the client)
        device_id = context.peer()
        
        # Log device connection
        logger.info(f'Device connected: {device_id}')
        
        # Track device connection
        connection_time = datetime.now()
        ImageStreamerServicer._device_connections[device_id] = {
            'last_connection': connection_time,
            'status': 'connected'
        }
        
        try:
            for image_frame in request_iterator:
                # Convert compressed JPEG to OpenCV image
                nparr = np.frombuffer(image_frame.image_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                # Save frame with timestamp and source
                filename = os.path.join(self.output_dir, f'{device_id}_frame_{image_frame.timestamp}_{self.frame_count}.jpg')
                cv2.imwrite(filename, frame)
                
                # Update latest image for this source
                ImageStreamerServicer._latest_images[device_id] = {
                    'image': frame,
                    'path': filename,
                    'timestamp': image_frame.timestamp
                }
                
                # Update last connection time
                ImageStreamerServicer._device_connections[device_id]['last_connection'] = datetime.now()
                
                self.frame_count += 1

                # Yield a response
                yield image_stream_pb2.ServerResponse(
                    status=image_stream_pb2.ServerResponse.Status.OK,
                    message=f'Received frame from {device_id}'
                )
        
        except Exception as e:
            # Log any streaming errors
            logger.error(f'Streaming error from {device_id}: {str(e)}')
            
            # Update device status
            ImageStreamerServicer._device_connections[device_id]['status'] = 'disconnected'
            
            # Raise the exception to terminate the connection
            raise
        
        finally:
            # Log device disconnection
            logger.info(f'Device disconnected: {device_id}')

    @classmethod
    def get_latest_image(cls, source_id=None):
        """Returns the most recently received image for a specific source or the most recent overall.
        
        :param source_id: Optional identifier for a specific source. If None, returns the most recent image from any source.
        :return: Numpy array of the image or None if no images exist
        """
        if not cls._latest_images:
            return None
        
        if source_id:
            return cls._latest_images.get(source_id, {}).get('image')
        
        # If no specific source, return the most recent image
        return max(cls._latest_images.values(), key=lambda x: x.get('timestamp', 0))['image']

    @classmethod
    def get_latest_image_path(cls, source_id=None):
        """Returns the file path of the most recently received image for a specific source or the most recent overall.
        
        :param source_id: Optional identifier for a specific source. If None, returns the path of the most recent image.
        :return: File path of the image or None if no images exist
        """
        if not cls._latest_images:
            return None
        
        if source_id:
            return cls._latest_images.get(source_id, {}).get('path')
        
        # If no specific source, return the path of the most recent image
        return max(cls._latest_images.values(), key=lambda x: x.get('timestamp', 0))['path']

    @classmethod
    def list_sources(cls):
        """Returns a list of all sources that have sent images.
        
        :return: List of source identifiers
        """
        return list(cls._latest_images.keys())

    @classmethod
    def get_device_status(cls, device_id=None):
        """Get connection status for a specific device or all devices.
        
        :param device_id: Optional device identifier
        :return: Connection status dictionary
        """
        if device_id:
            return cls._device_connections.get(device_id, {})
        return cls._device_connections

    @classmethod
    def cleanup_stale_connections(cls):
        """Remove devices that have been disconnected longer than the max disconnection time."""
        current_time = datetime.now()
        stale_devices = [
            device_id for device_id, device_info in cls._device_connections.items()
            if (current_time - device_info.get('last_connection', current_time)) > cls._max_disconnection_time
        ]
        
        for device_id in stale_devices:
            logger.warning(f'Stale device connection detected: {device_id}')
            cls._device_connections.pop(device_id, None)
            cls._latest_images.pop(device_id, None)

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
