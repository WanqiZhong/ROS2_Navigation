import rclpy
from nav_interfaces.msg import WheelSpeed
from std_msgs.msg import Float32
import serial
import numpy as np

def main():
    rclpy.init()
    node = rclpy.create_node('wheel_speed_input_serial')

    serial_port = '/dev/ttyACM0'
    baud_rate = 115200

    ser = serial.Serial(serial_port, baud_rate)
    node.get_logger().info('Serial port is opening...')

    publisher = node.create_publisher(WheelSpeed, '/serial/wheel_speed_in', 30)

    while rclpy.ok():
        a = ser.read(1)
        if a[0] == 0xAA:
            node.get_logger().info('Read 0xAA')
            data = ser.read(24)
            if len(data) == 24 and data[0] == 0xFF and data[23] == 0xAA:
                byte1 = data[0]
                byte2 = data[1]
                byte3 = data[2]
                byte4 = data[3]
                byte5 = data[4]
                byte6 = data[5]
                byte7 = data[6]
                byte8 = data[7]
                byte9 = data[8]
                byte10 = data[9]
                byte11 = data[10]
                byte12 = data[11]
                byte13 = data[12]
                byte14 = data[13]
                byte15 = data[14]
                byte16 = data[15]
                byte17 = data[16]
                byte18 = data[17]
                byte19 = data[18] 
                byte20 = data[19] 
                byte21 = data[20] 
                byte22 = data[21] 
                byte23 = data[22] 
                byte24 = data[23] 

                speed1 = (byte2 << 8) | byte3
                speed2 = (byte4 << 8) | byte5
                joystick_x = (byte6 << 8) | byte7
                joystick_y = (byte8 << 8) | byte9
                sensor_1 = (byte10 << 8) | byte11
                sensor_2 = (byte12 << 8) | byte13
                sensor_3 = (byte14 << 8) | byte15
                sensor_4 = (byte16 << 8) | byte17

                return_speed1 = (byte18<<8)|byte19
                return_speed2 = (byte20<<8)|byte21
                return_neg_flag1 = byte22
                return_neg_flag2 = byte23

                if 2100 > joystick_x > 1900:
                    joystick_x = 2000
                if 2100 > joystick_y > 1900:
                    joystick_y = 2000

                if speed1 > 10000:
                    rclpy.logging.get_logger("serial_read").info('More than 10000 (ori): {}'.format(speed1))
                    speed1 = (65535 - speed1)
                elif speed1 > 0:
                    rclpy.logging.get_logger("serial_read").info('More than 0 (ori): {}'.format(speed1))
                    speed1 = -speed1
                else:
                    rclpy.logging.get_logger("serial_read").info('Less than 0: {}'.format(speed1))
                
                if speed2 > 10000:
                    speed2 = -(65535 - speed2)

                # You can publish the data here
                msg = WheelSpeed()
                msg.speed_l = float(speed1)
                msg.speed_r = float(speed2)
                publisher.publish(msg)

                # node.get_logger().info('Read from joystick: x: {}, y: {}'.format(joystick_x, joystick_y))
                
                node.get_logger().info('Read from serial port: left speed: {}, right speed: {}'.format(speed1, speed2))
                node.get_logger().info('Read from serial port: return speed 1: {}, return speed 2: {}'.format(return_speed1, return_speed2))
                node.get_logger().info('Read from serial port: return neg flag 1: {}, return neg flag 2: {}'.format(return_neg_flag1, return_neg_flag2))

    ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
