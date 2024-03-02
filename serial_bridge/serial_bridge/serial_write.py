import rclpy
from nav_interfaces.msg import WheelSpeed
import struct
import serial
import numpy as np
import rclpy.logging as logging
from rclpy.node import Node

class SerialWriteNode(Node):
    def __init__(self):
        super().__init__('serial_write_node')
        self.declare_parameter('serial_port', '/dev/motor')

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = 115200
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        self.get_logger().info(f"Serial port: {self.serial_port}, Baud rate: {self.baud_rate}")

        self.subscription = self.create_subscription(
            WheelSpeed,
            '/serial/wheel_speed_out',
            self.callback,
            1
        )

    def callback(self, msg):
        speed_l = float(msg.speed_l)
        speed_r = float(msg.speed_r)

        speed_l_uint16 = abs(int(speed_l))  
        speed_r_uint16 = abs(int(speed_r))

        is_left_negative = 0 if int(speed_l) < 0 else 1
        is_right_negative = 0 if int(speed_r) < 0 else 1

        left_speed_bytes_first_8 = (speed_l_uint16 >> 8) & 0xff
        left_speed_bytes_last_8 = speed_l_uint16 & 0xff
        right_speed_bytes_first_8 = (speed_r_uint16 >> 8) & 0xff
        right_speed_bytes_last_8 = speed_r_uint16 & 0xff

        self.ser.write(bytes([0xff]))
        self.ser.write(bytes([left_speed_bytes_first_8]))
        self.ser.write(bytes([left_speed_bytes_last_8]))
        self.ser.write(bytes([right_speed_bytes_first_8]))
        self.ser.write(bytes([right_speed_bytes_last_8]))
        self.ser.write(bytes([is_left_negative]))
        self.ser.write(bytes([is_right_negative]))
        self.ser.write(bytes([0xaa]))

        logging.get_logger('wheel_speed_output_serial').info(f"Received float32: {msg.speed_l}, {msg.speed_r}, positive: {is_left_negative}, {is_right_negative}, Sent uint16: {speed_l_uint16}, {speed_r_uint16}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialWriteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
