import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoSerialListener(Node):
    def __init__(self):
        super().__init__('arduino_serial_listener')
        self.publisher_ = self.create_publisher(String, 'arduino_data', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)
            self.get_logger().info(f'Received: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
