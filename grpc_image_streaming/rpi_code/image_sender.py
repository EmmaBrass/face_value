import grpc
import cv2
import time
import numpy as np
import sys
sys.path.append('/path/to/proto/directory')  # Update this path on RPi

import image_stream_pb2
import image_stream_pb2_grpc

class ImageSender:
    def __init__(self, server_address='main_computer_ip:50051'):
        self.channel = grpc.insecure_channel(server_address)
        self.stub = image_stream_pb2_grpc.ImageStreamerStub(self.channel)

    def stream_frames(self, camera_index=0):
        cap = cv2.VideoCapture(camera_index)
        
        def frame_generator():
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                # Compress frame to JPEG
                _, buffer = cv2.imencode('.jpg', frame)
                
                yield image_stream_pb2.ImageFrame(
                    image_data=buffer.tobytes(),
                    timestamp=int(time.time() * 1000),
                    camera_id='rpi_camera_1'
                )
                time.sleep(0.1)  # Control frame rate

        try:
            responses = self.stub.StreamFrames(frame_generator())
            for response in responses:
                print(f'Server response: {response.message}')
        except grpc.RpcError as e:
            print(f'RPC error: {e}')
        finally:
            cap.release()

def main():
    sender = ImageSender()
    sender.stream_frames()

if __name__ == '__main__':
    main()
