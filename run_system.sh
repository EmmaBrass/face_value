#!/bin/bash

# Set environment variables
export PROJECT_ROOT="/Users/Emma/Documents/Documents_MacBook_Pro/pen_plotter/RoL 2025/rol_2025/face_value"
export PYTHONPATH="${PROJECT_ROOT}:$PYTHONPATH"

# Color codes for logging
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default mode is real (not simulation)
MODE="simulation"

# Parse command-line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -s|--simulation) MODE="simulation"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Function to kill all child processes
cleanup() {
    echo -e "${YELLOW}Stopping all processes...${NC}"
    pkill -P $$
    exit 0
}

# Trap signals to ensure cleanup
trap cleanup SIGINT SIGTERM

# Function to run in real mode
run_real_mode() {
    echo -e "${GREEN}Running in REAL mode${NC}"
    
    # Start image receiver
    echo -e "${YELLOW}Starting image receiver...${NC}"
    python "${PROJECT_ROOT}/face_value/grpc_image_streaming/server/image_receiver.py" &
    sleep 2  # Give server time to start
    
    # Start main application
    echo -e "${YELLOW}Starting main application...${NC}"
    python "${PROJECT_ROOT}/face_value/main.py" &
    
    # Wait for all background processes
    wait
}

# Function to run in simulation mode
run_simulation_mode() {
    echo -e "${GREEN}Running in SIMULATION mode${NC}"
    
    # Start image receiver
    echo -e "${YELLOW}Starting image receiver...${NC}"
    python "${PROJECT_ROOT}/face_value/grpc_image_streaming/server/image_receiver.py" &
    sleep 2  # Give server time to start
    
    # Start test image sender
    echo -e "${YELLOW}Starting test image sender...${NC}"
    python "${PROJECT_ROOT}/face_value/grpc_image_streaming/test_images/test_image_sender.py" &
    
    # Start main application
    echo -e "${YELLOW}Starting main application...${NC}"
    python "${PROJECT_ROOT}/face_value/main.py" &
    
    # Wait for all background processes
    wait
}

# Main execution
if [ "$MODE" == "simulation" ]; then
    run_simulation_mode
else
    run_real_mode
fi
