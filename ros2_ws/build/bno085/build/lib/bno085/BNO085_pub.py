import struct
import traceback
from math import radians, sin, cos

import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class BNO085_RVC_Publisher(Node):
    def __init__(self):
        super().__init__('BNO085_Publisher')
        self.imu_data_publisher = self.create_publisher(Imu, 'IMU_Data', 10)
        self.robot_orientation_publisher = self.create_publisher(Vector3, 'Robot_Euler_Orientation', 10)

        self.uart = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)
        self.get_logger().info('BNO085 RVC mode connected on /dev/ttyAMA0 at 115200 baud')

        self.read_send_timer = self.create_timer(0.01, self.read_and_send_imu_data)

    def euler_to_quaternion(self, roll_deg, pitch_deg, yaw_deg):
        roll = radians(roll_deg)
        pitch = radians(pitch_deg)
        yaw = radians(yaw_deg)

        cr = cos(roll / 2)
        sr = sin(roll / 2)
        cp = cos(pitch / 2)
        sp = sin(pitch / 2)
        cy = cos(yaw / 2)
        sy = sin(yaw / 2)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return x, y, z, w

    def read_rvc_packet(self):
        while True:
            b = self.uart.read(1)
            if len(b) == 0:
                return None
            if b[0] == 0xAA:
                b2 = self.uart.read(1)
                if len(b2) == 0:
                    return None
                if b2[0] == 0xAA:
                    break

        data = self.uart.read(17)
        if len(data) < 17:
            return None

        yaw, pitch, roll, x_accel, y_accel, z_accel = struct.unpack_from('<hhhhhh', data, 1)

        GRAVITY = 9.80665
        return {
            'yaw': yaw / 100.0,
            'pitch': pitch / 100.0,
            'roll': roll / 100.0,
            'x_accel': x_accel / 1000.0 * GRAVITY,
            'y_accel': y_accel / 1000.0 * GRAVITY,
            'z_accel': z_accel / 1000.0 * GRAVITY,
        }

    def read_and_send_imu_data(self):
        try:
            packet = self.read_rvc_packet()
            if packet is None:
                return

            qx, qy, qz, qw = self.euler_to_quaternion(
                packet['roll'], packet['pitch'], packet['yaw']
            )

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01

            imu_msg.linear_acceleration.x = packet['x_accel']
            imu_msg.linear_acceleration.y = packet['y_accel']
            imu_msg.linear_acceleration.z = packet['z_accel']
            imu_msg.linear_acceleration_covariance[0] = 0.1
            imu_msg.linear_acceleration_covariance[4] = 0.1
            imu_msg.linear_acceleration_covariance[8] = 0.1

            # RVC mode has no gyro data
            imu_msg.angular_velocity_covariance[0] = -1.0

            euler_msg = Vector3()
            euler_msg.x = packet['roll']
            euler_msg.y = packet['pitch']
            euler_msg.z = packet['yaw']

            self.imu_data_publisher.publish(imu_msg)
            self.robot_orientation_publisher.publish(euler_msg)

        except Exception as e:
            self.get_logger().warn(f"IMU read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BNO085_RVC_Publisher()
        rclpy.spin(node)
        node.destroy_node()
    except Exception as e:
        print(traceback.format_exc())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
