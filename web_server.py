from flask import Flask, request, send_from_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from flask import Response

app = Flask(__name__, static_folder='.')

# ROS2 Node wrapper
class ROS2Publisher(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.publisher_ = self.create_publisher(String, 'arduino_command', 10)

    def publish(self, msg):
        print(f"[ROS2Publisher] Attempting to publish: {msg}")
        try:
            msg_obj = String()
            msg_obj.data = msg
            self.publisher_.publish(msg_obj)
            print(f"[ROS2Publisher] Published: {msg}")
        except Exception as e:
            print(f"[ROS2Publisher] ERROR at publish: {e}")

# Start rclpy and node in a background thread
import threading
rclpy.init()
ros2_node = ROS2Publisher()
ros2_executor = rclpy.executors.SingleThreadedExecutor()
ros2_executor.add_node(ros2_node)
def spin_ros():
    ros2_executor.spin()
ros_thread = threading.Thread(target=spin_ros, daemon=True)
ros_thread.start()

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/<path:path>')
def static_files(path):
    return send_from_directory('.', path)

@app.route('/command/<cmd>', methods=['POST'])
def send_command(cmd):
    print(f"[Flask] Received POST /command/{cmd}")
    try:
        ros2_node.publish(cmd)
    except Exception as e:
        print(f"[Flask] ERROR at publish: {e}")
    return {'status': 'OK'}

def run_server():
    app.run(host='0.0.0.0', port=8080)

if __name__ == '__main__':
    run_server()
    
def gen_frames():
    cap = cv2.VideoCapture('/dev/video0')
    while True:
        success, frame = cap.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
