import threading
import socket
import logging
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string
from flask_socketio import SocketIO
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Logging-Setup
def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[logging.StreamHandler(sys.stdout)]
    )

# Helper: lokale IP ermitteln
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip

# HTML-Template mit Joystick-UI (nippleJS)
HTML = '''
<!doctype html>
<html lang="de">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ROS2 Stream & Joystick Control</title>
  <script src="https://cdn.jsdelivr.net/npm/nipplejs@0.7.3/dist/nipplejs.min.js"></script>
  <style>
    body, html { margin:0; padding:0; width:100%; height:100%; background:#000; }
    #stream { width:100%; height:auto; display:block; }
    #joystick-container { position:absolute; bottom:20px; right:20px; width:300px; height:300px; } }
  </style>
</head>
<body>
  <img id="stream" src="/video_feed" alt="Live Stream">
  <div id="joystick-container"></div>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.4.1/socket.io.min.js"></script>
  <script>
    const socket = io();
    const options = {
      zone: document.getElementById('joystick-container'),
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: 'white',
      size: 300
    };
    const joystick = nipplejs.create(options)[0];
    joystick.on('move', (evt, data) => {
      if (!data.angle || !data.distance) return;
      const angle = data.angle.radian;
      const dist = Math.min(data.distance, options.size / 2);
      const norm = dist / (options.size / 2);
      const turn = Number((Math.cos(angle) * norm).toFixed(2)); // angular.z
      const forward = Number((Math.sin(angle) * norm).toFixed(2)); // linear.x
      socket.emit('cmd_vel', { x: turn, y: forward });
    });
    joystick.on('end', () => {
      socket.emit('cmd_vel', { x: 0, y: 0 });
    });
  </script>
</body>
</html>
'''

# Flask und SocketIO Setup
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
node = None

# FrameBuffer für MJPEG-Streaming
class FrameBuffer:
    def __init__(self):
        self.condition = threading.Condition()
        self.frame = None

    def update(self, data: bytes):
        with self.condition:
            self.frame = data
            self.condition.notify_all()

    def get(self) -> bytes:
        with self.condition:
            self.condition.wait()
            return self.frame

frame_buffer = FrameBuffer()

# ROS2-Node für Bildempfang und cmd_vel-Publishing
class RosNode(Node):
    def __init__(self):
        super().__init__('ros_stream_joystick')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.get_logger().info('ROS2 Node gestartet')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ret, jpeg = cv2.imencode('.jpg', cv_image)
            if ret:
                frame_buffer.update(jpeg.tobytes())
        except Exception as e:
            self.get_logger().error(f'Image Error: {e}')

    def publish_cmd(self, x: float, y: float):
        twist = Twist()
        twist.linear.x = y * 0.5
        twist.angular.z = x * 1.0
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'CmdVel sent: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

# Flask-Routen
@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            frame = frame_buffer.get()
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('cmd_vel')
def handle_cmd(data):
    global node
    try:
        turn = float(data.get('x', 0))
        forward = float(data.get('y', 0))
    except ValueError:
        return
    logging.info(f'Received joystick: turn={turn}, forward={forward}')
    if node:
        node.publish_cmd(turn, forward)
    else:
        logging.warning('ROS-Node nicht initialisiert')

# Einstiegspunkt
def main():
    setup_logging()
    rclpy.init()
    global node
    node = RosNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    host = get_local_ip()
    port = 8000
    msg = f"=== SERVER RUNNING AT http://{host}:{port} ==="
    print(msg)
    sys.stdout.flush()
    logging.info(msg)

    try:
        socketio.run(app, host='0.0.0.0', port=port)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
