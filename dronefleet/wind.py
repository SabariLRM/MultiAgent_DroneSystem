"""A smooth, seeded spatio-temporal wind field for continuous flight.

    w(x, y, z, t) = profile(z) * (mean + sum_i a_i d_i sin(k_i . (x, y) - omega_i t + phi_i))

The mean wind blows toward ``wind_dir_deg``. Each gust mode has its own
horizontal direction ``d_i``, a wavelength of 400-2000 m and a period of
20-120 s, so gusts are correlated in space and time (a drone flying through
a gust feels it for tens of seconds, and its neighbours feel much the same).
With ``n`` modes of equal amplitude ``a = gust * sqrt(2 / n)`` the RMS gust
speed is ``wind_gust``. The vertical profile is a power law (wind shear):
``profile(z) = (z / wind_ref_height_m) ** wind_shear``.

The field has its own random stream, so it never perturbs the city, the
order stream or any other random draw of the simulation.
"""

from __future__ import annotations

import math
import random

# name -> (mean m/s, RMS gust m/s) at 30 m. "severe" is about the limit at which the
# energy-safe fleet still flies (see docs/REPORT.md)
WIND_PRESETS = {"calm": (0.0, 0.0), "moderate": (5.0, 1.5), "strong": (8.0, 2.5), "severe": (10.0, 3.0)}


class WindField:
    def __init__(self, cfg):
        rng = random.Random(cfg.seed * 7349 + 11)
        th = math.radians(cfg.wind_dir_deg)
        self.mean_speed = cfg.wind_mean
        self.gust = cfg.wind_gust
        self.mx = cfg.wind_mean * math.cos(th)
        self.my = cfg.wind_mean * math.sin(th)
        self.z_ref = cfg.wind_ref_height_m
        self.shear = cfg.wind_shear
        n = cfg.wind_modes if cfg.wind_gust > 0 else 0
        amp = cfg.wind_gust * math.sqrt(2.0 / n) if n else 0.0
        self.modes: list[tuple[float, ...]] = []
        for _ in range(n):
            d = rng.uniform(0.0, 2 * math.pi)            # direction the gust pushes
            kd = rng.uniform(0.0, 2 * math.pi)           # direction the wave travels
            k = 2 * math.pi / rng.uniform(400.0, 2000.0)
            omega = 2 * math.pi / rng.uniform(20.0, 120.0)
            self.modes.append((amp * math.cos(d), amp * math.sin(d), k * math.cos(kd), k * math.sin(kd),
                               omega, rng.uniform(0.0, 2 * math.pi)))
        self._prof: dict[int, float] = {}
        self.calm = not self.modes and self.mean_speed == 0.0

    def profile(self, z: float) -> float:
        """Wind-speed multiplier at height ``z`` (m); 1 at the reference height."""
        k = int(z)
        p = self._prof.get(k)
        if p is None:
            p = (max(float(k), 5.0) / self.z_ref) ** self.shear
            self._prof[k] = p
        return p

    def at(self, x: float, y: float, z: float, t: float) -> tuple[float, float]:
        """Horizontal wind vector (m/s) at position (m) and time (s)."""
        if self.calm:
            return 0.0, 0.0
        gx = gy = 0.0
        for ax, ay, kx, ky, om, ph in self.modes:
            s = math.sin(kx * x + ky * y - om * t + ph)
            gx += ax * s
            gy += ay * s
        p = self.profile(z)
        return (self.mx + gx) * p, (self.my + gy) * p

    def mean_at(self, z: float) -> tuple[float, float]:
        """The forecast (mean) wind at height ``z``: what the agents plan with."""
        p = self.profile(z)
        return self.mx * p, self.my * p

    def gust_at(self, z: float) -> float:
        """RMS gust speed at height ``z``."""
        return self.gust * self.profile(z)
