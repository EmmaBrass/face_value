FROM python:3.11-slim

# Install dependencies required by opencv-python
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV
RUN pip install opencv-python

# Copy your Python app
COPY app.py /app.py

CMD ["python", "/app.py"]

