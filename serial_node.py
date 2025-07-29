import rclpy
from rclpy.node import Node
import serial
import threading
import sys
import termios
import tty
import time

class KeyboardSerialNode(Node):
    def __init__(self):
        super().__init__('keyboard_serial_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.get_logger().info('Serial connected to Arduino.')
        self.running = True

        self.keyboard_thread = threading.Thread(target=self.read_keyboard)
        self.keyboard_thread.start()

        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.start()

    def read_keyboard(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:
                key = sys.stdin.read(1)
                if key == 'b':
                    self.send_to_arduino('black\n')
                elif key == 'w':
                    self.send_to_arduino('white\n')
                elif key == 's':
                    self.get_logger().info('Sending "start" rapidly for 10 seconds...')
                    threading.Thread(target=self.send_for_duration, args=('start\n', 10, 10)).start()
                elif key == 'r':
                    self.send_to_arduino('ok\n')
                elif key == 'q':
                    self.get_logger().info('Quit signal received.')
                    self.running = False
                    rclpy.shutdown()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def send_to_arduino(self, message):
        self.ser.write(message.encode())
        self.get_logger().info(f'Sent: {message.strip()}')

    
    def send_for_duration(self, message, duration, frequency):
        interval = 1.0 / frequency  
        end_time = time.time() + duration
        while time.time() < end_time and self.running:
            self.send_to_arduino(message)
            time.sleep(interval)
 

    def read_serial(self):
        while self.running:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                if response:
                    self.get_logger().info(f'Received from Arduino: {response}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSerialNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
