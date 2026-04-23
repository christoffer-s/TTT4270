"""
Mock av get_gps() uten serial/GPS.

- Returnerer samme type som originalen: np.array([lat, lon])
- Simulerer bevegelse i ENU (øst/nord) og konverterer til lat/lon
- Legger på litt støy (meter) for å ligne GPS

Bruk:
  python -m KalmanFilter.mock_gps
eller i app.py:
  from KalmanFilter.mock_gps import get_gps
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
import numpy as np


# Startpunkt (samme område som du bruker ellers)
ORIGO_LON = 10.402332799157428
ORIGO_LAT = 63.41809573255258

_METERS_PER_DEG_LAT = 111_111.0  # ca (god nok for små avstander)


def _meters_to_latlon_delta(east_m: float, north_m: float, lat_deg: float) -> tuple[float, float]:
    """Konverter (øst,nord) meter -> (dlat, dlon) grader ved gitt breddegrad."""
    dlat = north_m / _METERS_PER_DEG_LAT
    meters_per_deg_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(lat_deg))
    dlon = east_m / meters_per_deg_lon if meters_per_deg_lon != 0 else 0.0
    return dlat, dlon


@dataclass
class MockGPS:
    lat0: float = ORIGO_LAT
    lon0: float = ORIGO_LON
    vel_east_mps: float = 0.8    # m/s øst
    vel_north_mps: float = 0.2   # m/s nord
    noise_sigma_m: float = 1.5   # ca GPS-støy i meter (1-sigma)
    dt_s: float = 1.0

    def __post_init__(self) -> None:
        self._t = 0.0
        self._lat = float(self.lat0)
        self._lon = float(self.lon0)

    def read(self) -> np.ndarray:
        """Returnerer np.array([lat, lon]) tilsvarende get_gps()."""
        self._t += self.dt_s

        # Bevegelse (meter)
        east = self.vel_east_mps * self.dt_s
        north = self.vel_north_mps * self.dt_s

        # Litt "sving" så det ikke blir helt rett linje
        east += 0.3 * math.sin(self._t / 10.0)
        north += 0.2 * math.cos(self._t / 12.0)

        # Støy (meter)
        east += float(np.random.normal(0.0, self.noise_sigma_m))
        north += float(np.random.normal(0.0, self.noise_sigma_m))

        # Konverter til grader og oppdater posisjon
        dlat, dlon = _meters_to_latlon_delta(east, north, self._lat)
        self._lat += dlat
        self._lon += dlon

        return np.array([self._lat, self._lon], dtype=float)


# Global mock-instans + funksjon med samme navn/signatur som du bruker i appen
_mock = MockGPS()


def get_gps() -> np.ndarray:
    return _mock.read()


if __name__ == "__main__":
    while True:
        lat, lon = get_gps()
        print(f"{lat:.8f}, {lon:.8f}")
        time.sleep(_mock.dt_s)