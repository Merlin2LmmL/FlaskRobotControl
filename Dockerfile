FROM ros:humble

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-cv-bridge \
    python3-opencv \
    libglib2.0-0 \
    && pip3 install flask flask-socketio eventlet

WORKDIR /app
COPY app.py .

CMD ["python3", "app.py"]
