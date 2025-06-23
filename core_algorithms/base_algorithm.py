# BaseAlgorithm class to support EKF, UKF, TRANSFORMER,...etc integration.

class BaseAlgorithm:
    def __init__(self):
        self.state = None
        self.initialized = False

    def initialize(self, initial_state):
        self.state = initial_state
        self.initialized = True

    def update(self, *args, **kwargs):
        raise NotImplementedError("update() must be implemented in subclasses")

