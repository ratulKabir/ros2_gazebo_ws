#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')

        # Data buffers
        self.truth = {'x': [], 'y': []}
        self.noisy = {'x': [], 'y': []}
        self.ekf   = {'x': [], 'y': []}
        self.ego_x, self.ego_y = 0.0, 0.0

        # Subscribers
        self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.truth_cb, 10)
        self.create_subscription(Odometry, '/odom_noisy', self.noisy_cb, 10)
        self.create_subscription(Odometry, '/ekf/odom', self.ekf_cb, 10)

        # Plot setup
        self.fig, self.ax = plt.subplots()
        self.noisy_plot, = self.ax.plot([], [], 'ro', label='Noisy', markersize=3)
        self.ekf_plot,   = self.ax.plot([], [], 'b-', label='EKF')
        self.truth_plot, = self.ax.plot([], [], 'g-', label='Ground Truth')
        self.ax.legend()
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Live Trajectory Comparison (Centered on Ego)')
        self.ax.grid(True)
        self.ax.axis('equal')

        # Fixed view parameters
        self.view_range = 5.0  # meters around ego

    # ---------------- Callbacks ----------------
    def truth_cb(self, msg):
        self.truth['x'].append(msg.pose.pose.position.x)
        self.truth['y'].append(msg.pose.pose.position.y)
        # use truth as ego reference (or EKF if you prefer)
        self.ego_x, self.ego_y = self.truth['x'][-1], self.truth['y'][-1]

    def noisy_cb(self, msg):
        self.noisy['x'].append(msg.pose.pose.position.x)
        self.noisy['y'].append(msg.pose.pose.position.y)

    def ekf_cb(self, msg):
        self.ekf['x'].append(msg.pose.pose.position.x)
        self.ekf['y'].append(msg.pose.pose.position.y)

    # ---------------- Plot Update ----------------
    def update_plot(self, frame):
        self.truth_plot.set_data(self.truth['x'], self.truth['y'])
        self.noisy_plot.set_data(self.noisy['x'], self.noisy['y'])
        self.ekf_plot.set_data(self.ekf['x'], self.ekf['y'])

        # Keep the plot centered on the ego position
        if len(self.truth['x']) > 0:
            cx, cy = self.ego_x, self.ego_y
            r = self.view_range
            self.ax.set_xlim(cx - r, cx + r)
            self.ax.set_ylim(cy - r, cy + r)

        return self.truth_plot, self.noisy_plot, self.ekf_plot


def main(args=None):
    rclpy.init(args=args)
    node = LivePlotter()

    # Spin ROS2 in background thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Live animation
    ani = FuncAnimation(node.fig, node.update_plot, interval=200)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
