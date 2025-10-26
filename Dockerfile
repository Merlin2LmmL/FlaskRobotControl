# ─── Base image ───
FROM ubuntu:22.04

# ─── Arguments & environment ───
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=$ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV FLASK_ENV=production

# ─── Install system dependencies ───
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release \
    python3 python3-pip python3-venv \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# ─── Add ROS 2 apt repository ───
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# ─── Install ROS 2 + Python dependencies ───
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# ─── Set working directory ───
WORKDIR /app

# ─── Copy Python requirements & install them first (for caching) ───
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# ─── Copy the rest of the project ───
COPY . .

# ─── Expose Flask port ───
EXPOSE 8000

# ─── Launch script: dynamically source ROS and run app ───
ENTRYPOINT ["/bin/bash", "-lc", "\
  if [ -z \"$ROS_DISTRO\" ]; then ROS_DISTRO=humble; fi; \
  if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then \
      source /opt/ros/$ROS_DISTRO/setup.bash; \
  else \
      echo 'Warning: ROS distro setup not found, defaulting to Humble'; \
      source /opt/ros/humble/setup.bash; \
  fi; \
  echo 'Running FlaskRobotControl with ROS distro: '$ROS_DISTRO; \
  python3 /app/app.py \
"]
