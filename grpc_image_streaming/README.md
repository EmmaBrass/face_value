# gRPC Image Streaming

## Setup

### Main Computer
1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Generate gRPC code:
```bash
python -m grpc_tools.protoc -I./proto --python_out=. --grpc_python_out=. ./proto/image_stream.proto
```

3. Run server:
```bash
python server/image_receiver.py
```

### Raspberry Pi
1. Install dependencies
2. Update `server_address` in `image_sender.py`
3. Run sender:
```bash
python rpi_code/image_sender.py
```

## Notes
- Adjust frame rate and compression as needed
- Ensure network connectivity between devices
- Add error handling for production use
