# plusgins/algos/ekf_node.py

import math
from typing import Optional, Dict, Any
import numpy as np
from plugins.base_node import BaseNode
from core_algorithms.EKF.BaseEKF import EKFAlgorithm

R_EARTH = 6371000.0

def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2*math.pi) - math.pi

class EKFNode(BaseNode):
    """
    Predict: IMU yaw_rate  gyro z-axis）and velocity v
    Update:  GNSS（lat/lon to x/y) or magnetometer（mx/my to yaw_mag）
    No wheel speed. From GNSS to find approximate v
    """
    def __init__(self,
                 x0 = [0,0,0], P0 = [0.5,0.5,0.5],
                 R_gnss = [1.0,1.0], R_mag = [0.05],
                 dt_min: float = 1e-3, dt_max: float = 0.2,
                 use_mag: bool = True, declination_deg: float = 0.0,
                 # source column map - CSV rename
                 map_imu = {"v":"v","gz":"wz"},
                 map_gnss= {"lat":"lat","lon":"lon"},
                 map_mag = {"mx":"mx","my":"my"}):
        self.dt_min, self.dt_max = dt_min, dt_max
        self.use_mag = use_mag
        self.decl = math.radians(declination_deg)

        self.map_imu, self.map_gnss, self.map_mag = map_imu, map_gnss, map_mag

        self.ekf = EKFAlgorithm(np.array(x0,float), np.diag(P0))
        self.ekf.set_observation_noise("gnss", np.diag(R_gnss))
        self.ekf.set_observation_noise("magnetometer", np.array([[R_mag if isinstance(R_mag,(int,float)) else R_mag[0]]],float))

        # time and init position
        self.last_t: Optional[float] = None
        self.lat0 = self.lon0 = None
        self.cos_lat0 = None
        self.prev_xy: Optional[tuple] = None
        self.cached_v: Optional[float] = None

    def reset(self):
        self.last_t = None
        self.lat0 = self.lon0 = None
        self.cos_lat0 = None
        self.prev_xy = None
        self.cached_v = None

    def _latlon_to_xy(self, lat_deg: float, lon_deg: float):
        lat = math.radians(lat_deg); lon = math.radians(lon_deg)
        if self.lat0 is None:
            self.lat0, self.lon0 = lat, lon
            self.cos_lat0 = math.cos(self.lat0)
        x = (lon - self.lon0) * self.cos_lat0 * R_EARTH
        y = (lat - self.lat0) * R_EARTH
        return x, y

    def _mag_heading(self, mx: float, my: float):
        # simple version and no adjustment. Right hand z-axis up and object face to x-axis 
        return wrap_angle(math.atan2(-my, mx) + self.decl)

    def on_event(self, ev: Dict[str, Any]):
        t = float(ev["timestamp"])
        topic = ev["_topic"]

        if self.last_t is None:
        	dt = 0
        else:
            dt =  (t - self.last_t)
            if dt <= 0:
                dt = 0
            else:
                dt = min(self.dt_max, max(self.dt_min, dt))
            
        if topic == "imu":
            # 1. User external velocity 
            # 2. GNSS position difference to get velocity 
            # 3. Or just assume velocity = 0
            v = ev.get(self.map_imu.get("v","v"), None)
            if v is None:
                v = self.cached_v if self.cached_v is not None else 0.0
            gz = float(ev.get(self.map_imu.get("gz","wz"), 0.0))
            u = np.array([float(v), gz], float)
            self.ekf.predict(u, dt, model="imu")

        elif topic == "gnss" and (self.map_gnss["lat"] in ev and self.map_gnss["lon"] in ev):
            x, y = self._latlon_to_xy(float(ev[self.map_gnss["lat"]]),
                                      float(ev[self.map_gnss["lon"]]))
            # Update
            self.ekf.update(np.array([x,y],float), "gnss")
            
            if self.prev_xy is not None and self.last_t is not None:
                px, py, pt = self.prev_xy
                dtv = max(1e-6, t - pt)
                self.cached_v = math.hypot(x - px, y - py) / dtv
            self.prev_xy = (x, y, t)

        elif self.use_mag and topic == "mag" and (self.map_mag["mx"] in ev and self.map_mag["my"] in ev):
            yaw_mag = self._mag_heading(float(ev[self.map_mag["mx"]]),
                                        float(ev[self.map_mag["my"]]))
            self.ekf.update(np.array([yaw_mag],float), "magnetometer")

        self.last_t = t
        x = self.ekf.get_state()
        return {"timestamp": t, "x": float(x[0]), "y": float(x[1]), "yaw": float(wrap_angle(x[2]))}

