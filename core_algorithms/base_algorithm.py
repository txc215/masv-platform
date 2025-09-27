import numpy as np

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


def mag_to_heading(mx, my, declination_deg=0.0, wrap_to_2pi=True):
    """
    magnetometer X/Y to heading in radians。
    add declination angles in degree
    """
    heading = np.arctan2(my, mx)
    heading += np.deg2rad(declination_deg)

    if wrap_to_2pi:
        heading = heading % (2 * np.pi)
    return heading

def gnss_to_heading(lat1, lon1, lat2, lon2, wrap_to_2pi=True):
    """
    base on 2 lat and 2 lon to find heading (rad)
    """
    d_lon = np.radians(lon2 - lon1)
    lat1 = np.radians(lat1)
    lat2 = np.radians(lat2)

    x = np.sin(d_lon) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(d_lon)

    heading = np.arctan2(x, y)
    if wrap_to_2pi:
        heading = heading % (2 * np.pi)
    return heading

