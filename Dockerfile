# ─── Base image (Ubuntu 22.04 compatible with ROS2) ───
FROM ubuntu:22.04

# ─── Build argument (default auto-detect at build time) ───
ARG ROS_DISTRO
# Use Humble as fallback if not provided
ENV ROS_DISTRO=${ROS_DISTRO:-$(bash -c "source /opt/ros/*/setup.bash 2>/dev/null || true && echo $ROS_DISTRO" || echo "humble")}

# ─── Install base dependencies ───
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release python3 python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

# ─── Add ROS 2 apt repository ───
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# ─── Install system + ROS dependencies (auto-using $ROS_DISTRO) ───
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# ─── Working directory ───
WORKDIR /app

# ─── Copy project and install Python dependencies ───
COPY FlaskRobotControl/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# ─── Copy the rest of your Flask-ROS2 app ───
COPY FlaskRobotControl/ .

# ─── Environment setup ───
ENV FLASK_ENV=production
ENV PYTHONUNBUFFERED=1

# ─── Expose Flask port ───
EXPOSE 8000

# ─── Launch script: source the correct ROS setup dynamically ───
ENTRYPOINT ["/bin/bash", "-lc", "\
  if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then \
    source /opt/ros/$ROS_DISTRO/setup.bash; \
  else \
    echo 'Warning: ROS distro setup not found, defaulting to /opt/ros/humble'; \
    source /opt/ros/humble/setup.bash; \
  fi && \
  python3 /app/app.py \
"]
