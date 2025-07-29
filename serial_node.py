import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        self.pub = self.create_publisher(String, 'arduino_response', 10)
        self.sub = self.create_subscription(String, 'arduino_command', self.command_callback, 10)

        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

    def command_callback(self, msg):
        cmd = msg.data + '\n'
        self.ser.write(cmd.encode('utf-8'))
        self.get_logger().info(f'Sent to Arduino: {msg.data}')

    def read_serial(self):
        while rclpy.ok():
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                self.get_logger().info(f'Received from Arduino: {line}')
                msg = String()
                msg.data = line
                self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
