# ─── Base: Official ROS2 Humble (Python 3.10, Ubuntu 22.04) ───
FROM ros:humble

# ─── System + ROS dependencies ───
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    ros-humble-rclpy \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# ─── Working directory ───
WORKDIR /app

# ─── Copy requirements and install Python packages ───
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# ─── Copy your Flask-ROS2 control app ───
COPY app.py .

# ─── Environment setup ───
ENV FLASK_ENV=production
ENV PYTHONUNBUFFERED=1

# ─── Expose web port ───
EXPOSE 8000

# ─── Start command ───
ENTRYPOINT ["/bin/bash", "-lc", "\
  source /opt/ros/humble/setup.bash && \
  python3 /app/app.py \
"]
