import rclpy
from rclpy.node import Node
import serial
import threading
import sys
import termios
import tty

class KeyboardSerialNode(Node):
    def __init__(self):
        super().__init__('keyboard_serial_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  
        self.get_logger().info('Serial connected to Arduino.')
        self.running = True

        # Thread pentru citirea de la tastatura
        self.keyboard_thread = threading.Thread(target=self.read_keyboard)
        self.keyboard_thread.start()

        # Thread pentru citirea de la Arduino
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
                elif key == 'q':
                    self.get_logger().info('Quit signal received.')
                    self.running = False
                    rclpy.shutdown()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def send_to_arduino(self, message):
        self.ser.write(message.encode())
        self.get_logger().info(f'Sent: {message.strip()}')

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
