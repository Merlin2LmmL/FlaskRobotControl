#!/usr/bin/env python3
import threading
import socket
import logging
import sys
import subprocess

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

# HTML-Template mit Joystick und zwei Servo-Slidern
HTML = '''
<!doctype html>
<html lang="de">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ROS2 Stream & Dual Servo Control</title>
  <script src="https://cdn.jsdelivr.net/npm/nipplejs@0.7.3/dist/nipplejs.min.js"></script>
  <style>
    body, html { margin:0; padding:0; width:100%; height:100%; background:#000; color:white; overflow:hidden; }
    #stream { width:100%; height:auto; display:block; }
    #joystick-container {
      position: absolute;
      bottom: 10px;
      right: 10px;
      width: 200px;
      height: 200px;
    }
    #sliders {
      position: absolute;
      bottom: 10px;
      left: 10px;
      background: rgba(0,0,0,0.6);
      padding: 10px;
      border-radius: 8px;
      display: flex;
      flex-direction: column;
      gap: 20px;
    }
    .slider-group label { display:block; margin-bottom:5px; }
  </style>
</head>
<body>
  <img id="stream" src="/video_feed" alt="Live Stream">
  <div id="joystick-container"></div>
  <div id="sliders">
    <div class="slider-group">
      <label for="servo1-slider">Servo1-Winkel: <span id="servo1-value">0</span>°</label>
      <input type="range" id="servo1-slider" min="-150" max="150" value="0" style="width:200px;" />
    </div>
    <div class="slider-group">
      <label for="servo2-slider">Servo2-Winkel: <span id="servo2-value">0</span>°</label>
      <input type="range" id="servo2-slider" min="-150" max="150" value="0" style="width:200px;" />
    </div>
  </div>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.4.1/socket.io.min.js"></script>
  <script>
    const socket = io();
    // Joystick Setup (continuous send while moving)
    const options = {
      zone: document.getElementById('joystick-container'),
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: 'white',
      size: 200
    };
    const joystick = nipplejs.create(options)[0];
    joystick.on('move', (evt, data) => {
      if (!data.angle || !data.distance) return;
      const angle = data.angle.radian;
      const dist = Math.min(data.distance, options.size / 2);
      const norm = dist / (options.size / 2);
      const forward = Number((-Math.cos(angle) * norm).toFixed(2));
      const turn = Number((Math.sin(angle) * norm).toFixed(2));
      socket.emit('cmd_vel', { x: turn, y: forward });
    });
    joystick.on('end', () => {
      socket.emit('cmd_vel', { x: 0, y: 0 });
    });
    // Slider für Servo1
    const s1 = document.getElementById('servo1-slider');
    const v1 = document.getElementById('servo1-value');
    s1.addEventListener('change', () => {
      const angle = parseInt(s1.value);
      v1.textContent = angle;
      socket.emit('servo_cmd', { servo: 'servo1', angle: angle });
    });
    // Slider für Servo2
    const s2 = document.getElementById('servo2-slider');
    const v2 = document.getElementById('servo2-value');
    s2.addEventListener('change', () => {
      const angle = parseInt(s2.value);
      v2.textContent = angle;
      socket.emit('servo_cmd', { servo: 'servo2', angle: angle });
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
    def __init__(self): self.condition = threading.Condition(); self.frame = None
    def update(self, data: bytes):
        with self.condition: self.frame = data; self.condition.notify_all()
    def get(self) -> bytes:
        with self.condition: self.condition.wait(); return self.frame
frame_buffer = FrameBuffer()

# ROS2-Node
class RosNode(Node):
    def __init__(self):
        super().__init__('ros_stream_control')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))
        self.get_logger().info('ROS2 Node gestartet')
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            ret, jpeg = cv2.imencode('.jpg', cv_image)
            if ret: frame_buffer.update(jpeg.tobytes())
        except Exception as e: self.get_logger().error(f'Image Error: {e}')
    def publish_cmd(self, x: float, y: float):
        twist = Twist(); twist.linear.x = y*0.5; twist.angular.z = x*1.0; self.cmd_pub.publish(twist)
        self.get_logger().info(f'CmdVel sent: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

# Flask-Routen
@app.route('/')
def index(): return render_template_string(HTML)
@app.route('/video_feed')
def video_feed():
    def gen():
        while True: yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_buffer.get() + b'\r\n')
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('cmd_vel')
def handle_cmd(data):
    node.publish_cmd(float(data['x']), float(data['y']))

@socketio.on('servo_cmd')
def handle_servo(data):
    servo = data['servo']; angle = int(data['angle'])
    topic = f'/{servo}/angle_cmd'
    subprocess.run(['ros2','topic','pub','--once',topic,'std_msgs/msg/Int32',f'{{data: {angle}}}'])
    logging.info(f'Published {angle} to {topic}')

# Main
def main():
    setup_logging(); rclpy.init();
    global node; node = RosNode(); threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    host = get_local_ip(); port = 8000
    print(f"=== SERVER RUNNING AT http://{host}:{port} ==="); logging.info("Server gestartet")
    try: socketio.run(app, host='0.0.0.0', port=port)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
