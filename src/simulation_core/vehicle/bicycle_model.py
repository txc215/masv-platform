class BicycleModel:
    def __init__(self, wheelbase=2.5):
        self.L = wheelbase
        self.state = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
    def update(self, v, delta, dt):
        x, y, theta = self.state['x'], self.state['y'], self.state['theta']
        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt
        dtheta = (v / self.L) * math.tan(delta) * dt
        self.state['x'] += dx
        self.state['y'] += dy
        self.state['theta'] += dtheta
        return self.state.copy()