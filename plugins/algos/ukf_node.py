# plugins/algos/ukf_node.py

import math
from typing import Optional, Dict, Any
import numpy as np
from plugins.base_node import BaseNode
from core_algorithms.UKF.ukf import UKFAlgorithm

R_EARTH = 6371000.0

def _wrap(a: float) -> float: 
    return (a + math.pi) % (2*math.pi) - math.pi

class UKFNode(BaseNode):
    """
    Predict: IMU yaw_rate（gz） velocity v(external or difference of GNSS position)
    Update:  GNSS(x,y) or Mag(yaw)
    """
    def __init__(self,
                 x0=[0,0,0], 
                 P0=[[1,0,0],
                     [0,1,0],
                     [0,0,0.1]],
                 Q=[[0.01,0,0],
                    [0,0.01,0],
                    [0,0,0.001]],
                 R_gnss=[[1,0],
                         [0,1]], 
                 R_mag=[[0.05]],
                 alpha=1e-3, 
                 beta=2.0, 
                 kappa=0,
                 dt_min=1e-3, 
                 dt_max=0.2,
                 use_mag=True, 
                 declination_deg=0.0,
                 map_imu={"v":"v","gz":"wz"},
                 map_gnss={"lat":"lat","lon":"lon"},
                 map_mag={"mx":"mx","my":"my"}):
        self.dt_min, self.dt_max = dt_min, dt_max
        self.use_mag = use_mag
        self.decl = math.radians(declination_deg)
        self.map_imu, self.map_gnss, self.map_mag = map_imu, map_gnss, map_mag

        def f_model(x: np.ndarray, dt: float) -> np.ndarray:
            # from self.ukf.u get v and gz
            xk = x.copy()
            yaw = xk[2]
            v  = self.ukf.u[0] if self.ukf.u is not None else 0.0
            gz = self.ukf.u[1] if self.ukf.u is not None else 0.0
            xk[0] += v * math.cos(yaw) * dt
            xk[1] += v * math.sin(yaw) * dt
            xk[2] = _wrap(xk[2] + gz * dt)
            return xk

        def h_gnss(x): 
            return x[0:2]

        def h_mag(x):  
            return np.array([_wrap(x[2])])

        self.ukf = UKFAlgorithm(
            f=f_model,
            h_dict={"gnss": h_gnss, "magnetometer": h_mag},
            Q=np.array(Q,float),
            R_dict={"gnss": np.array(R_gnss,float), "magnetometer": np.array(R_mag,float)},
            alpha=alpha, beta=beta, kappa=kappa
        )
        self.ukf.initialize(np.array(x0,float), np.array(P0,float), timestamp=0.0)

        self.last_t: Optional[float] = None
        self._last_pred_t: Optional[float] = None  # every time_t only has one data
        self.lat0: Optional[float] = None
        self.lon0: Optional[float] = None
        self.cos_lat0: Optional[float] = None
        self.prev_xy: Optional[tuple] = None  # (x, y, t) for v estimation
        self.cached_v: Optional[float] = None


    # ---------------- tools ----------------
    def reset(self) -> None:
        self.last_t = None
        self._last_pred_t = None
        self.lat0 = self.lon0 = self.cos_lat0 = None
        self.prev_xy = None
        self.cached_v = None

    def _latlon_to_xy(self, lat_deg: float, lon_deg: float) -> tuple:
        """small range equirectangular：WGS-84 approimate to surface（meter）"""
        lat = math.radians(float(lat_deg)); lon = math.radians(float(lon_deg))
        if self.lat0 is None:
            self.lat0, self.lon0 = lat, lon
            self.cos_lat0 = math.cos(self.lat0)
        x = (lon - self.lon0) * (self.cos_lat0 or 1.0) * R_EARTH
        y = (lat - self.lat0) * R_EARTH
        return x, y

    def _yaw_from_mag(self, mx: float, my: float) -> float:
        """
        Find heading from magnetometer mx, my. No hard-iron and soft-iron adjustment
        atan2(-my, mx) + declination
        """
        return _wrap(math.atan2(-float(my), float(mx)) + self.decl)

    def _maybe_predict_to(self, t_now: float) -> None:
        """same timestamp propagate once. rest measurement just update。"""
        if self._last_pred_t != t_now:
            self.ukf.predict(timestamp=t_now)
            self._last_pred_t = t_now

    # ---------------- main event  ----------------
    def on_event(self, ev: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Format：
          IMU: {"_topic":"imu",  "timestamp":t, "wz":..., or {"gz":..., "v":...}}
               or column name：{"gyro_z_radps":..., "accel_x_mps2":...} setup by map_imu
          GNSS:{"_topic":"gnss", "timestamp":t, "lat":..., "lon":...} or {"x":...,"y":...}
          MAG: {"_topic":"mag",  "timestamp":t, "mx":..., "my":...} or {"yaw":...}
        return
          {"timestamp": t, "x":..., "y":..., "yaw":...}
        """
        t = float(ev["timestamp"])
        topic = ev["_topic"]

        # ---------- predict ----------
        if topic == "imu":
            # Velecity: use external v first. If not, use GNSS latitude and longtitude. Or just zero if no previous information
            v_key = self.map_imu.get("v")
            v = ev.get(v_key) if v_key is not None else None
            if v is None:
                v = self.cached_v if self.cached_v is not None else 0.0

            gz_key = self.map_imu.get("gz", "wz")
            gz = float(ev.get(gz_key, 0.0))
            self.ukf.set_input(np.array([float(v), gz], dtype=float))
            # propagate to t - do once in one timestamp
            self._maybe_predict_to(t)

        else:
            # non IMU event also propagate state into time t if hasn't propagate
            self._maybe_predict_to(t)

        # ---------- Update predict ----------
        if topic == "gnss":
            # Support lat/lon or just go x/y
            if self.map_gnss.get("x") in ev and self.map_gnss.get("y") in ev:
                x = float(ev[self.map_gnss["x"]])
                y = float(ev[self.map_gnss["y"]])
            elif self.map_gnss.get("lat") in ev and self.map_gnss.get("lon") in ev:
                x, y = self._latlon_to_xy(ev[self.map_gnss["lat"]], ev[self.map_gnss["lon"]])
            else:
                x = y = None

            if x is not None and y is not None:
                self.ukf.update("gnss", np.array([x, y], dtype=float))
                # GNSS divide difference to find approximate velocity for next IMU event
                if self.prev_xy is not None:
                    px, py, pt = self.prev_xy
                    dtv = max(1e-6, t - pt)
                    self.cached_v = math.hypot(x - px, y - py) / dtv
                self.prev_xy = (x, y, t)

        elif self.use_mag and (self.map_mag.get("yaw") in ev or
                               (self.map_mag.get("mx") in ev and self.map_mag.get("my") in ev)):
            # use yaw directly or from mx/my
            if self.map_mag.get("yaw") in ev:
                yaw_meas = wrap_angle(float(ev[self.map_mag["yaw"]]))
            else:
                mx = ev[self.map_mag["mx"]]; my = ev[self.map_mag["my"]]
                yaw_meas = self._yaw_from_mag(mx, my)
            self.ukf.update("magnetometer", np.array([yaw_meas], dtype=float))

        # ---------- Ending ----------
        self.last_t = t  # memorize last process time t
        x, _P = self.ukf.get_state()
        return {"timestamp": t, "x": float(x[0]), "y": float(x[1]), "yaw": float(_wrap(x[2]))}


