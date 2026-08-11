"""
Utility functions for processing and manipulating GNSS data.

This module wraps RTKLIB functionality to load and manipulate
GNSS observation and navigation files from RINEX-compatible
receivers.

Author: Sammy Guo
Date: 2026-01-28
"""

from dataclasses import dataclass
import gzip
from pathlib import Path
import shutil
import tempfile
import urllib.request

from IPython.display import IFrame
import numpy as np
import pyrtklib as rtklib


@dataclass
class SatellitePositions:
    n: int                   # Number of observations.
    rs: rtklib.Arr1Ddouble   # Satellite positions and velocities:
                             # rs [(0:2)+i*6] = obs[i] sat position {x,y,z} (m)
                             # rs [(3:5)+i*6] = obs[i] sat velocity {vx,vy,vz} (m/s)
    dts: rtklib.Arr1Ddouble  # Satellite clock drift:
                             # dts[(0:1)+i*2] = obs[i] sat clock {bias,drift} (s|s/s)
    var: rtklib.Arr1Ddouble  # Satellite position and clock variances:
                             # var[i] = obs[i] sat position and clock error variance (m^2)
    svh: rtklib.Arr1Dint     # Satellite health flag (-1:correction not available):
                             # svh[i] = obs[i] sat health flag

    def __init__(self, n: int):
        self.n = n
        self.rs = rtklib.Arr1Ddouble(6*n)
        self.dts = rtklib.Arr1Ddouble(2*n)
        self.var = rtklib.Arr1Ddouble(1*n)
        self.svh = rtklib.Arr1Dint(1*n)

    def satPosition(self, i: int) -> np.ndarray:
        return np.array([
            self.rs[6*i+0], 
            self.rs[6*i+1], 
            self.rs[6*i+2]
        ])

    def satClockBias(self, i: int) -> float:
        return self.dts[2*i+0]

    def satClockDrift(self, i: int) -> float:
        return self.dts[2*i+1]


@dataclass
class RINEXData:
    obs: rtklib.obs_t
    nav: rtklib.nav_t
    sta: rtklib.sta_t

    def __init__(self):
        self.obs = rtklib.obs_t()
        self.nav = rtklib.nav_t()
        self.sta = rtklib.sta_t()

    def __str__(self):
        return f"Found {self.obs.n} observations\nFound {self.nav.n} navigation messages"

    def loadRINEX(self, path: str) -> int:
        return rtklib.readrnx(path, 1, "", self.obs, self.nav, self.sta)

    def computeSatelliteOrbits(self, teph: rtklib.gtime_t) -> SatellitePositions:
        result = SatellitePositions(self.obs.n)
        rtklib.satposs(
            teph,                     # gtime_t teph     I   time to select ephemeris (gpst)
            self.obs.data.ptr,        # obsd_t *obs      I   observation data
            self.obs.n,               # int    n         I   number of observation data
            self.nav,                 # nav_t  *nav      I   navigation data
            0,                        # int    ephopt    I   ephemeris option (EPHOPT_???) (EPHOPT_BRDC == 0)
            result.rs,                # double *rs       O   satellite positions and velocities (ecef)
            result.dts,               # double *dts      O   satellite clocks
            result.var,               # double *var      O   sat position and clock error variances (m^2)
            result.svh                # int    *svh      O   sat health flag (-1:correction not available)
        )
        return result


def loadCORSRINEX(url_paths: list[str]) -> RINEXData:
    """Downloads and parses a list of URLs from CORS online dataset.
    """
    # Create a temporary directory (auto-deleted when closed):
    result = RINEXData()
    for p in url_paths:
        with tempfile.TemporaryDirectory() as tmpdir:
            tmpdir = Path(tmpdir)
            
            compressed_file = tmpdir / Path(p).name
            uncompressed_file = tmpdir / Path(p).stem
        
            # Download files using built-in HTTP library:
            with urllib.request.urlopen(p) as response, open(compressed_file, "wb") as f:
                f.write(response.read())
        
            # Extract files:
            with gzip.open(compressed_file, "rb") as gz, open(uncompressed_file, "wb") as out:
                shutil.copyfileobj(gz, out)
        
            # Parse and load data:
            load_status: int = result.loadRINEX(str(uncompressed_file))
            assert load_status == 1, f"Unable to load file {p}"
    return result

def plotMap(lat, lon):
    osm_url = f"https://www.openstreetmap.org/export/embed.html?bbox={lon-0.01}%2C{lat-0.01}%2C{lon+0.01}%2C{lat+0.01}&layer=map&marker={lat}%2C{lon}"
    return IFrame(osm_url, width=700, height=500)

def iterateObservations(recv_data: RINEXData, sat_positions: SatellitePositions, pr_code: int = rtklib.CODE_L1C, n: int = -1):
    for i in range(n):
        obsd = recv_data.obs.data[i]
        sat_bias = sat_positions.satClockBias(i)
        sat_pos = sat_positions.satPosition(i)
    
        # Skip this observation if sat positioning failed:
        if np.linalg.norm(sat_pos) < 1.0:
            continue
    
        # Identify pseudorange code CODE_L1C:
        for j, code in enumerate(obsd.code):
            if code == pr_code:
                pseudorange = obsd.P[j]
                break
        else:
            # If no CODE_L1C pseudorange found, skip this observation:
            continue

        yield obsd, sat_bias, sat_pos, pseudorange