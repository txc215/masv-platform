import numpy as np
import sys

sys.path.insert(0, '../')
from base_algorithm import BaseAlgorithm, mag_to_heading

class UKFAlgorithm(BaseAlgorithm):
    def __init__(self, f, h_dict, Q, R_dict, alpha=1e-3, beta=2.0, kappa=0):
        super().__init__()
        self.f = f            # Process model function
        self.h_dict = h_dict  # Measurement model function
        self.Q = Q            # Process noise covariance
        self.R_dict = R_dict  # Measurement noise covariance

        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa

        self.P = None  # Covariance
        self.n = None
        self.lmbda = None
        self.Wm = None  # Weights for mean
        self.Wc = None  # Weights for covariance

        self.u = None

        self.last_timestamp = None
        self.sigma_points_pred = None
        self.x_pred = None
        self.P_pred = None

    def initialize(self, initial_state, initial_cov, timestamp):
        super().initialize(initial_state)
        self.P = initial_cov
        self.n = initial_state.shape[0]

        self.lmbda = self.alpha**2 * (self.n + self.kappa) - self.n
        self.gamma = np.sqrt(self.n + self.lmbda)

        # Calculate weights
        self.Wm = np.full(2 * self.n + 1, 1 / (2 * (self.n + self.lmbda)))
        self.Wc = np.full(2 * self.n + 1, 1 / (2 * (self.n + self.lmbda)))
        self.Wm[0] = self.lmbda / (self.n + self.lmbda)
        self.Wc[0] = self.Wm[0] + (1 - self.alpha**2 + self.beta)
        self.last_timestamp = timestamp


    def generate_sigma_points(self, x, P):
        sigma_points = np.zeros((2 * self.n + 1, self.n))
        sigma_points[0] = x
        S = np.linalg.cholesky((self.n + self.lmbda) * P)
        for i in range(self.n):
            sigma_points[i + 1]          = x + S[:, i]
            sigma_points[i + 1 + self.n] = x - S[:, i]
        return sigma_points

    def predict(self, timestamp):
        assert self.state is not None, "UKF state is not initialized!"
        dt = (timestamp - self.last_timestamp)
        self.last_timestamp = timestamp

        sigma_points = self.generate_sigma_points(self.state, self.P)
        self.sigma_points_pred = np.array([self.f(sp, dt) for sp in sigma_points])
        self.x_pred = np.sum(self.Wm[:, None] * self.sigma_points_pred, axis=0)
        self.P_pred = self.Q.copy()

        for i in range(2 * self.n + 1):
            dx = self.sigma_points_pred[i] - self.x_pred
            self.P_pred += self.Wc[i] * np.outer(dx, dx)

    def update(self, sensor_name, z):
        assert self.state is not None, "UKF state is not initialized!"
        if self.sigma_points_pred is None:
            return
        h = self.h_dict[sensor_name]
        R = self.R_dict[sensor_name]

        Z_sigma = np.array([h(sp) for sp in self.sigma_points_pred])
        z_pred = np.sum(self.Wm[:, None] * Z_sigma, axis=0)

        P_zz = R.copy()
        for i in range(2 * self.n + 1):
            dz = Z_sigma[i] - z_pred
            P_zz += self.Wc[i] * np.outer(dz, dz)

        P_xz = np.zeros((self.n, z.shape[0]))
        for i in range(2 * self.n + 1):
            dx = self.sigma_points_pred[i] - self.x_pred
            dz = Z_sigma[i] - z_pred
            P_xz += self.Wc[i] * np.outer(dx, dz)

        K = P_xz @ np.linalg.inv(P_zz)
        self.state = self.x_pred + K @ (z - z_pred)
        self.P = self.P_pred - K @ P_zz @ K.T

    def get_state(self, keys=None):
        if keys is not None:
            return self.state[keys], self.P[np.ix_(keys, keys)]
        return self.state.copy(), self.P.copy()

    def set_input(self, u):
        self.u = u



if __name__ == '__main__':

    def h_f(x):
        return x

    # def fun(x, gyro_z_radps, dt):
    #     return x + gyro_z_radps * dt;
    def fun(x, dt):
        return x

    h_dc = {"mag": h_f}

    R_dc = {"mag": np.array([[0.01]])}

    ukf = UKFAlgorithm(f=fun, h_dict=h_dc, Q=np.array([[1e-4]]), R_dict=R_dc)

    time_t = 0

    ukf.initialize(initial_state=np.array([[0.0]]), initial_cov=np.array([[0.1]]), timestamp = time_t)

    dt = 1.0

    # simulate data：gyro_z = 0.05 rad/s，observe heading + noise
    
    true_heading = 0.0
    for t in range(10):
        time_t += dt
        true_heading += 0.05 * dt
        measured_heading = true_heading + np.random.normal(0, 0.01)

        ukf.predict(time_t)
        ukf.update("mag", z=np.array([[measured_heading]]))

        print(f"Step {t+1}, Estimate = {ukf.state[0][0]:.3f}, True = {true_heading:.3f}")
