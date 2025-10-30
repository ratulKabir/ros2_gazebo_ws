#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # Publishers / Subscribers
        self.create_subscription(Odometry, '/odom_noisy', self.meas_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/ekf/odom', 10)

        # EKF parameters
        self.dt = 0.1
        self.x = np.zeros(3)  # [x, y, yaw]
        self.P = np.diag([1.0, 1.0, np.deg2rad(10)**2])

        self.Q = np.diag([0.02, 0.02, np.deg2rad(2.0)])**2  # process noise
        self.R = np.diag([0.5, 0.5, np.deg2rad(5.0)])**2  # measurement noise

        self.v = 0.0
        self.w = 0.0

        # Predict on timer
        self.timer = self.create_timer(self.dt, self.predict)

    # ---------------------------------------------------------
    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    # ---------------------------------------------------------
    def predict(self):
        theta = self.x[2]
        v, w = self.v, self.w
        dt = self.dt

        # Nonlinear motion model
        self.x[0] += v * math.cos(theta) * dt
        self.x[1] += v * math.sin(theta) * dt
        self.x[2] += w * dt
        self.x[2] = self.wrap_angle(self.x[2])

        # Jacobian of motion model wrt state
        F = np.array([
            [1, 0, -v * math.sin(theta) * dt],
            [0, 1,  v * math.cos(theta) * dt],
            [0, 0, 1]
        ])

        self.P = F @ self.P @ F.T + self.Q

        # publish continuously
        self.publish_estimate()

    # ---------------------------------------------------------
    def meas_callback(self, msg: Odometry):
        # Extract noisy measurement
        z = np.zeros(3)
        z[0] = msg.pose.pose.position.x
        z[1] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        z[2] = math.atan2(siny_cosp, cosy_cosp)

        # Measurement model: z = Hx + noise
        H = np.eye(3)
        y = z - self.x
        y[2] = self.wrap_angle(y[2])
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update
        self.x = self.x + K @ y
        self.x[2] = self.wrap_angle(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P

        self.publish_estimate()

    # ---------------------------------------------------------
    def publish_estimate(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])

        q = self.quaternion_from_euler(0, 0, float(self.x[2]))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Flatten covariance matrix
        cov = np.zeros(36)
        cov[0] = self.P[0, 0]
        cov[7] = self.P[1, 1]
        cov[35] = self.P[2, 2]
        msg.pose.covariance = list(cov)

        self.pub_odom.publish(msg)

    # ---------------------------------------------------------
    @staticmethod
    def wrap_angle(a):
        return (a + np.pi) % (2*np.pi) - np.pi
    

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w).
        The angles should be in radians.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
