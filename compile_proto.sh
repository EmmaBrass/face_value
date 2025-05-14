#!/bin/bash

# Set the project root
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Protobuf file location
PROTO_FILE="${PROJECT_ROOT}/grpc_image_streaming/proto/image_stream.proto"

# Output directory for compiled files
OUTPUT_DIR="${PROJECT_ROOT}"

# Compile the protobuf
python3 -m grpc_tools.protoc \
    -I"${PROJECT_ROOT}" \
    --python_out="${OUTPUT_DIR}" \
    --grpc_python_out="${OUTPUT_DIR}" \
    "${PROTO_FILE}"

# Create an __init__.py to make it a proper package
touch "${OUTPUT_DIR}/__init__.py"

echo "Protobuf compilation complete."
