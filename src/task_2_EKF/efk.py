# ============================================================
# Extended Kalman Filter (EKF) — 2D Robot Tracking Example
# ============================================================
# A simple but conceptually correct EKF implementation that tracks
# a robot’s (x, y) position and velocity using noisy range-bearing
# measurements to a single landmark.
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass


# ------------------------------------------------------------
# 1. Configuration container
# ------------------------------------------------------------
@dataclass
class EKFConfig:
    dt: float = 0.1                 # time step (s)
    q_pos: float = 0.05             # process noise std for position
    q_vel: float = 0.05             # process noise std for velocity
    r_range: float = 0.5            # measurement noise std for range
    r_bearing: float = np.deg2rad(2) # measurement noise std for bearing (radians)
    landmark: tuple = (0.0, 0.0)    # (x, y) of known landmark


# ------------------------------------------------------------
# 2. Extended Kalman Filter class
# ------------------------------------------------------------
class ExtendedKalmanFilter:
    """
    EKF for a constant-velocity 2D motion model.
    State vector: x = [px, py, vx, vy]^T
    Measurement: range and bearing to a known landmark.
    """

    def __init__(self, cfg: EKFConfig):
        self.cfg = cfg

        # --- Linear motion model (constant velocity) ---
        dt = cfg.dt
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ], dtype=float)

        # --- Covariances ---
        qx2 = cfg.q_pos**2
        qv2 = cfg.q_vel**2
        self.Q = np.diag([qx2, qx2, qv2, qv2])   # process noise
        self.R = np.diag([cfg.r_range**2, cfg.r_bearing**2])  # measurement noise

        # --- Initialize state and covariance ---
        self.x = np.zeros(4)
        self.P = np.eye(4) * 10.0

    # --------------------------------------------------------
    # Initialize state
    # --------------------------------------------------------
    def set_initial(self, x0, P0=None):
        self.x = x0.astype(float)
        self.P = np.eye(4) if P0 is None else P0.astype(float)

    # --------------------------------------------------------
    # Prediction step (linear)
    # --------------------------------------------------------
    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    # --------------------------------------------------------
    # Nonlinear measurement function h(x)
    # --------------------------------------------------------
    def h(self, x):
        Lx, Ly = self.cfg.landmark
        px, py, vx, vy = x
        dx = px - Lx
        dy = py - Ly
        rng = np.sqrt(dx**2 + dy**2)
        bearing = np.arctan2(dy, dx)
        return np.array([rng, bearing])

    # --------------------------------------------------------
    # Jacobian of measurement function
    # --------------------------------------------------------
    def H_jacobian(self, x):
        Lx, Ly = self.cfg.landmark
        px, py, vx, vy = x
        dx = px - Lx
        dy = py - Ly
        q = dx**2 + dy**2
        r = np.sqrt(q) + 1e-12  # avoid division by zero

        # Partial derivatives
        dr_dpx = dx / r
        dr_dpy = dy / r
        dth_dpx = -dy / q
        dth_dpy =  dx / q

        # Only position affects the measurement
        H = np.array([
            [dr_dpx,  dr_dpy, 0.0, 0.0],
            [dth_dpx, dth_dpy, 0.0, 0.0]
        ])
        return H

    # --------------------------------------------------------
    # Helper: wrap angles into [-pi, pi]
    # --------------------------------------------------------
    @staticmethod
    def wrap_angle(phi):
        return (phi + np.pi) % (2 * np.pi) - np.pi

    # --------------------------------------------------------
    # Update step (nonlinear, uses linearized H)
    # --------------------------------------------------------
    def update(self, z):
        # Predict measurement
        z_pred = self.h(self.x)
        H = self.H_jacobian(self.x)

        # Innovation
        y = z - z_pred
        y[1] = self.wrap_angle(y[1])  # normalize bearing difference

        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.x = self.x + K @ y
        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P


# ------------------------------------------------------------
# 3. Simulate ground-truth trajectory + noisy measurements
# ------------------------------------------------------------
def simulate_truth_and_measurements(cfg: EKFConfig, steps=250, seed=42):
    np.random.seed(seed)
    dt = cfg.dt
    Lx, Ly = cfg.landmark

    x_true = np.zeros((steps, 4))
    z_meas = np.zeros((steps, 2))
    # Start somewhere near the landmark with nonzero velocity
    x = np.array([8.0, -2.0, 1.0, 0.6], dtype=float)

    for k in range(steps):
        # simple curved trajectory via small oscillating acceleration
        ax = 0.1 * np.cos(0.05 * k)
        ay = 0.1 * np.sin(0.05 * k)

        # propagate true motion
        x[0] += x[2] * dt
        x[1] += x[3] * dt
        x[2] += ax * dt
        x[3] += ay * dt
        x_true[k] = x

        # create noisy measurement (range + bearing)
        dx = x[0] - Lx
        dy = x[1] - Ly
        rng = np.sqrt(dx**2 + dy**2) + np.random.randn() * cfg.r_range
        bearing = np.arctan2(dy, dx) + np.random.randn() * cfg.r_bearing
        z_meas[k] = np.array([rng, bearing])

    return x_true, z_meas


# ------------------------------------------------------------
# 4. Run EKF on simulated data
# ------------------------------------------------------------
cfg = EKFConfig()
x_true, z_meas = simulate_truth_and_measurements(cfg)

ekf = ExtendedKalmanFilter(cfg)
ekf.set_initial(np.array([5.0, 5.0, 0.0, 0.0]), P0=np.eye(4)*5.0)

xs_est = []
for z in z_meas:
    ekf.predict()
    ekf.update(z)
    xs_est.append(ekf.x.copy())

xs_est = np.array(xs_est)


# ------------------------------------------------------------
# 5. Helper: convert range/bearing to x/y for visualization
# ------------------------------------------------------------
def rb_to_xy(r, b, Lx, Ly):
    return Lx + r * np.cos(b), Ly + r * np.sin(b)

meas_xy = np.array([rb_to_xy(r, b, *cfg.landmark) for r, b in z_meas])

# ------------------------------------------------------------
# 6. Visualization
# ------------------------------------------------------------
plt.figure()
plt.plot(x_true[:, 0], x_true[:, 1], label="True trajectory")
plt.plot(xs_est[:, 0], xs_est[:, 1], label="EKF estimate")
plt.scatter(meas_xy[:, 0], meas_xy[:, 1], s=8, label="Noisy measurements")
plt.scatter([cfg.landmark[0]], [cfg.landmark[1]], marker="x", c="k", label="Landmark")
plt.title("2D Tracking with EKF (Range-Bearing Sensor)")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()
plt.axis("equal")
plt.tight_layout()
plt.show()

# --- Error plot ---
pos_err = np.linalg.norm(x_true[:, :2] - xs_est[:, :2], axis=1)
plt.figure()
plt.plot(pos_err)
plt.title("Position Error Norm over Time")
plt.xlabel("Time step")
plt.ylabel("Position error [m]")
plt.tight_layout()
plt.show()
