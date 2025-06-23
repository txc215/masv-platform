from abc import ABC, abstractmethod
import numpy as np

class BaseMotionModel(ABC):
    """
    Abstract base class for motion models.
    """

    @abstractmethod
    def predict(self, state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
        """
        Predict the next state given the current state and control input.

        :param state: Current state vector
        :param control: Control input vector
        :param dt: Time step
        :return: Predicted next state vector
        """
        pass