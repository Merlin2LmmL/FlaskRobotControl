# Basis-Image mit ROS2 Humble
FROM ros:humble

# System- & ROS-Abhängigkeiten
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-cv-bridge \
    python3-opencv \
    git \
  && pip3 install --no-cache-dir flask flask-socketio eventlet opencv-python \
  && rm -rf /var/lib/apt/lists/*

# Arbeitsverzeichnis für dein Flask-App
WORKDIR /app

# Deinen Flask-Code kopieren
COPY app.py .
COPY requirements.txt .

# (Optional, falls Du pip aus requirements.txt bevorzugst)
# RUN pip3 install --no-cache-dir -r requirements.txt

# ENTRYPOINT sourct ROS und dein Overlay, dann startet die App
ENTRYPOINT ["/bin/bash","-lc", "\
    source /opt/ros/humble/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    python3 /app/app.py \
"]
