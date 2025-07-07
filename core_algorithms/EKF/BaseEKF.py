# EKF-enhanced version of BaseAlgorithm

import numpy as np
import sys
import socket

hostname = socket.gethostname()

if 'Mac' in hostname or 'Windows' in hostname:
    sys.path.insert(0, '../../')
    from core_algorithms.base_algorithm import BaseAlgorithm
    from motion_models.vehicle.bicycle_model import BicycleModel
else:
    from base_algorithm import BaseAlgorithm
    from vehicle.bicycle_model import BicycleModel


class EKFAlgorithm(BaseAlgorithm):
    # def __init__(self, initial_state, initial_covariance, f_func, f_jacobian,
    #              h_models, Q, R_dict):
    #     self.x = initial_state              # init state vector
    #     self.P = initial_covariance         # covariance matrix
    #     self.f_func = f_func                # state transition function f(x, u, dt)
    #     self.f_jacobian = f_jacobian        # f's' Jacobian F(x, u, dt)
    #     self.h_models = h_models            # obervation model dict: {sensor_type: (h_func, H_func)}
    #     self.Q = Q                          # transition noise covariance matrix 
    #     self.R = R_dict                     # observation noise covariance matrix dict: {sensor_type: R_matrix}

    # def predict(self, u, dt):
    #     F = self.f_jacobian(self.x, u, dt)
    #     self.x = self.f_func(self.x, u, dt)
    #     self.P = F @ self.P @ F.T + self.Q

    # def update(self, z, sensor_type):
    #     h_func, H_func = self.h_models[sensor_type]
    #     H = H_func(self.x)
    #     y = z - h_func(self.x)
    #     S = H @ self.P @ H.T + self.R[sensor_type]
    #     K = self.P @ H.T @ np.linalg.inv(S)
    #     self.x = self.x + K @ y
    #     self.P = (np.eye(len(self.x)) - K @ H) @ self.P

    # def get_state(self):
    #     return self.x.copy()

    # def set_R(self, sensor_type, R_matrix):
    #     self.R[sensor_type] = R_matrix

    def __init__(self, initial_state, initial_covariance):
        super().__init__()
        self.x = initial_state
        self.P = initial_covariance
        self.Q = np.eye(len(self.x)) * 0.01
        self.R = {}
        self.initialized = True
        self.bike_mod = BicycleModel(2.5)

    def predict(self, u, dt, model="imu"):
        
        if model == "bicycle":
            F = self.bike_mod.jacobian(self.x.copy(), u, dt)
            self.x = self.bike_mod.predict(self.x.copy(), u, dt)
        else:
            F = self._compute_imu_jacobian_F(u, dt)
            self.x = self._motion_model(self.x, u, dt)
        self.P = F @ self.P @ F.T + self.Q


    def update(self, z, sensor_type):
        H, h_func = self._get_observation_model(sensor_type)
        y = z - h_func(self.x)
        S = H @ self.P @ H.T + self.R[sensor_type]
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P

    def get_state(self):
        return self.x.copy()

    def _motion_model(self, x, u, dt):
        x_new = x.copy()
        theta = x[2]
        v = u[0]
        yaw_rate = u[1]
        x_new[0] += v * np.cos(theta) * dt
        x_new[1] += v * np.sin(theta) * dt
        x_new[2] += yaw_rate * dt
        return x_new

    def _compute_imu_jacobian_F(self, u, dt):
        theta = self.x[2]
        v = u[0]
        F = np.eye(len(self.x))
        F[0, 2] = -v * np.sin(theta) * dt
        F[1, 2] =  v * np.cos(theta) * dt
        return F

    def _get_observation_model(self, sensor_type):
        if sensor_type == "gnss":
            def h(x): return x[0:2]
            H = np.zeros((2, len(self.x)))
            H[0, 0] = 1
            H[1, 1] = 1
        elif sensor_type == "magnetometer":
            def h(x): return np.array([x[2]])
            H = np.zeros((1, len(self.x)))
            H[0, 2] = 1
        else:
            raise ValueError("Unknown sensor type")
        return H, h

    def set_observation_noise(self, sensor_type, R_matrix):
        self.R[sensor_type] = R_matrix


    
