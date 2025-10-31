import numpy as np
import math
import matplotlib.pyplot as plt


class EKF:
    def __init__(self, dt=0.1,
                 Q=np.diag([0.02, 0.02, np.deg2rad(2.0)])**2,
                 R=np.diag([0.5, 0.5, np.deg2rad(5.0)])**2):
        """
        Extended Kalman Filter for a differential-drive robot.

        State: [x, y, yaw]
        """
        self.dt = dt
        self.Q = Q
        self.R = R
        self.x = np.zeros(3)
        self.P = np.diag([1.0, 1.0, np.deg2rad(10)**2])

    # -------------------------------------------------------------
    def predict(self, v, w):
        """Predict next state using motion model and control inputs."""
        theta = self.x[2]
        dt = self.dt

        # Motion model
        self.x[0] += v * math.cos(theta) * dt
        self.x[1] += v * math.sin(theta) * dt
        self.x[2] += w * dt
        self.x[2] = self._wrap_angle(self.x[2])

        # Jacobian wrt state
        F = np.array([
            [1, 0, -v * math.sin(theta) * dt],
            [0, 1,  v * math.cos(theta) * dt],
            [0, 0, 1]
        ])

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q
        self.P = 0.5 * (self.P + self.P.T)  # enforce symmetry

        return self.x.copy(), self.P.copy()

    # -------------------------------------------------------------
    def update(self, z):
        """Update step given a noisy measurement z = [x, y, yaw]."""
        H = np.eye(3)
        y = z - self.x
        y[2] = self._wrap_angle(y[2])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = self._wrap_angle(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

        return self.x.copy(), self.P.copy()

    # -------------------------------------------------------------
    @staticmethod
    def _wrap_angle(a):
        return (a + np.pi) % (2 * np.pi) - np.pi


# ===============================================================
# Standalone demo example with visualization
# ===============================================================
if __name__ == "__main__":
    ekf = EKF(dt=0.1)

    v_true = 1.0   # m/s
    w_true = 0.1   # rad/s

    true_path = []
    est_path = []
    meas_path = []

    x_true = np.zeros(3)

    for t in np.arange(0, 10, ekf.dt):
        # --- True motion ---
        x_true[0] += v_true * math.cos(x_true[2]) * ekf.dt
        x_true[1] += v_true * math.sin(x_true[2]) * ekf.dt
        x_true[2] += w_true * ekf.dt
        x_true[2] = ekf._wrap_angle(x_true[2])

        # --- Predict ---
        ekf.predict(v_true, w_true)

        # --- Noisy measurement ---
        z = x_true + np.random.multivariate_normal(mean=[0, 0, 0], cov=ekf.R)

        # --- Update ---
        est, P = ekf.update(z)

        true_path.append(x_true.copy())
        est_path.append(est.copy())
        meas_path.append(z.copy())

    true_path = np.array(true_path)
    est_path = np.array(est_path)
    meas_path = np.array(meas_path)

    # --- Plot results ---
    plt.figure(figsize=(6, 6))
    plt.plot(true_path[:, 0], true_path[:, 1], 'g-', label='True path')
    plt.plot(est_path[:, 0], est_path[:, 1], 'b-', label='EKF estimate')
    plt.scatter(meas_path[:, 0], meas_path[:, 1], s=10, c='r', alpha=0.5, label='Noisy measurements')

    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("EKF Localization Demo")
    plt.show()

    print("\nFinal estimate:")
    print("x =", ekf.x.round(3))
    print("P =", np.round(ekf.P, 3))
