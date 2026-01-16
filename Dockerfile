# ─── Base image: preinstalled ROS 2 Humble ───
FROM ros:humble-ros-base

# ─── Environment ───
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV FLASK_ENV=production
ENV ROS_DISTRO=humble

# ─── Install system & ROS runtime dependencies ───
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# ─── Working directory ───
WORKDIR /app

# ─── Install Python dependencies (cached layer) ───
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# ─── Copy application code ───
COPY . .

# ─── Expose Flask port ───
EXPOSE 8000

# ─── Start application ───
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && python3 /app/app.py"]
