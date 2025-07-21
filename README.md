# FluskRobotControl

A lightweight Flask‐and‐SocketIO based web interface for streaming a ROS 2 camera feed and controlling a robot via a virtual joystick. FluskRobotControl bridges ROS 2 (Humble) and the browser, exposing:

- **Live MJPEG stream** of the `/image_raw` sensor topic  
- **NippleJS‐powered on‑screen joystick** that publishes `geometry_msgs/Twist` messages to `/cmd_vel`  
- Seamless integration via `cv_bridge` and `rclpy`  
- Docker support for one‑command deployment

---

## Features

- **Real‑time video streaming**  
  Converts incoming `sensor_msgs/Image` messages into JPEG frames, served over HTTP as an MJPEG stream.

- **Web joystick control**  
  A browser‑embedded joystick (using [NippleJS](https://github.com/yoannmoinet/nipplejs)) that emits normalized X/Y axes to the Flask server, which translates them into linear and angular velocities.

- **ROS 2 Humble compatibility**  
  Uses `rclpy` for ROS 2 node management, QoS configuration, and publishing/subscription.

- **Containerized deployment**  
  Official ROS 2 Humble Docker image as base, installs all dependencies and runs the app out of the box.

---

## Prerequisites

- **ROS 2 Humble** installed (or Docker Desktop)  
- Python 3.8+  
- (Optional) Docker & Docker Compose

---

## Installation

1. **Clone the repository**  
   ```bash
   git clone https://github.com/Merlin2LmmL/FluskRobotControl.git
   cd FluskRobotControl
