# **FlaskRobotControl**

A lightweight Flask-and-SocketIO based web interface for streaming a ROS 2 camera feed and controlling a robot via a virtual joystick and servo sliders. FlaskRobotControl bridges ROS 2 (Humble) and the browser, exposing:

* **Live MJPEG stream** of the `/image_raw` sensor topic
* **NippleJS-powered on-screen joystick** that publishes `geometry_msgs/Twist` messages to `/cmd_vel`
* **Servo control sliders** that publish `std_msgs/Int32` messages to `/<servo_name>/angle_cmd`
* Seamless integration via `cv_bridge` and `rclpy`
* Docker support for one-command deployment

---

## Features

* **Real-time video streaming**
  Converts incoming `sensor_msgs/Image` messages into JPEG frames, served over HTTP as an MJPEG stream.

* **Web joystick control**
  A browser-embedded joystick (using [NippleJS](https://github.com/yoannmoinet/nipplejs)) that emits normalized X/Y axes to the Flask server, which translates them into linear and angular velocities and publishes `geometry_msgs/Twist` on `/cmd_vel`.

* **Servo control via sliders**
  Interactive HTML sliders in the web UI allow users to adjust angles for configured servos. Slider values (`Int32`) are published to `/<servo_name>/angle_cmd`, which the [servo\_helpers](https://github.com/Merlin2LmmL/servo-helpers) node relays to the Studica `set_servo_angle` service endpoint.

* **ROS 2 Humble compatibility**
  Uses `rclpy` for ROS 2 node management, QoS configuration, and publishing/subscription.

* **Containerized deployment**
  Official ROS 2 Humble Docker image as base, installs all dependencies and runs the app out of the box.

---

## Prerequisites

* **ROS 2 Humble** installed (or Docker Desktop)
* Python 3.8+
* (Optional) Docker & Docker Compose

---

## Installation

1. **Clone the repository**

   ```bash
   git clone https://github.com/Merlin2LmmL/FlaskRobotControl.git
   cd FlaskRobotControl
   ```

2. **Install Python dependencies**

   ```bash
   pip install -r requirements.txt
   ```

3. **Build and source your ROS 2 workspace** (if not using Docker)

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

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

### Local (bare metal)

```bash
export ROBOT_NAMESPACE=""
export CAMERA_TOPIC="/image_raw"
export JOYSTICK_TOPIC="/cmd_vel"
export SERVO_NAMES="servo1,servo2"
python3 app.py
```

### Docker

```bash
docker-compose up --build
```

Then open in browser: `http://localhost:5000`

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
