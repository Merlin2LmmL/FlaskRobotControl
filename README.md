# FlaskRobotControl

A lightweight **Flask + SocketIO web interface** for streaming a ROS 2 camera feed and controlling a robot via a virtual joystick and servo sliders. `FlaskRobotControl` bridges **any ROS 2 distro** and the browser, exposing:

- **Live MJPEG stream** of a ROS 2 camera topic (`/image_raw` by default)
- **NippleJS-powered on-screen joystick** publishing `geometry_msgs/Twist` messages to `/cmd_vel`
- **Servo control sliders** publishing `std_msgs/Int32` messages to `/<servo_name>/angle_cmd`
- Seamless integration via `cv_bridge` and `rclpy`
- Containerized deployment via **Docker** for reproducible environments

---

## Features

1. **Real-time video streaming**  
   Converts incoming `sensor_msgs/Image` messages into JPEG frames served over HTTP as an MJPEG stream.

2. **Web joystick control**  
   A browser-embedded joystick ([NippleJS](https://github.com/yoannmoinet/nipplejs)) emits normalized X/Y axes to the Flask server, which translates them into linear and angular velocities and publishes `geometry_msgs/Twist` on `/cmd_vel`.

3. **Servo control via sliders**  
   Interactive HTML sliders allow adjusting angles for configured servos. Slider values (`Int32`) are published to `/<servo_name>/angle_cmd` and relayed to the Studica robot via the [`servo_helpers`](https://github.com/Merlin2LmmL/servo-helpers) node.

4. **ROS 2 compatibility**  
   Supports **any ROS 2 distribution** (`humble`, `iron`, `jazzy`, etc.) using `rclpy` for node management, QoS configuration, and publishing/subscription.

5. **Containerized deployment**  
   Official Ubuntu 22.04 + ROS 2 Docker base image, installs all dependencies, and runs the app out of the box.

---

## Prerequisites

- **ROS 2** installed (any supported distro) or Docker Desktop
- Python 3.8+
- Docker & Docker Compose

---

## Installation

1. **Clone the repository**

   ```bash
   git clone https://github.com/Merlin2LmmL/FlaskRobotControl.git
   cd FlaskRobotControl
   ```

## Quickstart

To quickly start up the server, follow the full Docker instructions. Building the docker should not take more than a few seconds.

---   

## Configuration

Environment variables or command-line flags:

| Variable          | Default         | Description                                      |
| ----------------- | --------------- | ------------------------------------------------ |
| `ROBOT_NAMESPACE` | `''`            | ROS namespace for topics and services            |
| `CAMERA_TOPIC`    | `/image_raw`    | Topic for camera image streaming                 |
| `JOYSTICK_TOPIC`  | `/cmd_vel`      | Topic for Twist commands from the joystick       |
| `SERVO_NAMES`     | `servo1,servo2` | Comma-separated list of servos to expose sliders |

---

## Usage

### Docker

1. **Building Docker Image**

     ```bash
     cd FlaskRobotControl/
     docker build --build-arg ROS_DISTRO=${ROS_DISTRO} -t flask_robot_control . // Replace ros2 distro as needed
     ```

 2. **Running Container (With Camera Support)**

    ```bash
    docker run --rm -it   --net host   --device /dev/video0:/dev/video0   flask_robot_control
    ```

3. Then open in browser the logged address (typically: `http://localhost:5000`).

### Local (bare metal NOT RECOMMENDED)

```bash
export ROBOT_NAMESPACE=""
export CAMERA_TOPIC="/image_raw"
export JOYSTICK_TOPIC="/cmd_vel"
export SERVO_NAMES="servo1,servo2"
python3 app.py
```

---

## Slider Topics and Configuration

For each servo listed in `SERVO_NAMES`, the web UI generates a corresponding HTML `<input type="range">` slider with:

* **Label**: Displays current angle value in degrees.
* **Range**: Configured via `min="-150" max="150"` (adjustable as needed).
* **Event**: On slider `change`, the UI emits a Socket.IO event `servo_cmd` carrying `{ servo: '<servo_name>', angle: <value> }`.

The server handler in `app.py` captures these events and executes:

```bash
ros2 topic pub --once /<servo_name>/angle_cmd std_msgs/msg/Int32 "{data: <angle>}"
```

This publishes an `std_msgs/Int32` message on the topic `/<servo_name>/angle_cmd`.

The [`servo_helpers`](https://github.com/Merlin2LmmL/servo-helpers) node subscribes to these topics, translates incoming angle commands into `SetData` service requests, and calls `/<servo_name>/set_servo_angle` on the Studica hardware. See the repo for implementation details.

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
