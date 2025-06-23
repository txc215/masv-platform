from .base_model import BaseMotionModel
import numpy as np

class BicycleModel(BaseMotionModel):
    def __init__(self, wheelbase=2.5):
        self.L = wheelbase

        
    def predict(self, state, control_input, dt):
        x, y, theta = state
        v, delta = control_input
        dx = v * np.cos(theta) * dt
        dy = v * np.sin(theta) * dt
        dtheta = (v / self.L) * np.tan(delta) * dt
        return np.array([x + dx, y + dy, theta + dtheta])

    def jacobian(self, state, ctrl, dt):
        """
        Compute Jacobian of the bicycle model motion function w.r.t. state.
        state: [x, y, theta]
        control: [v, delta]
        """
        x, y, theta = state
        v, delta = ctrl

        F = np.eye(3)

        F[0, 2] = -v * np.sin(theta) * dt
        F[1, 2] =  v * np.cos(theta) * dt
        

        return F