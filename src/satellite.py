import numpy as np
from copy import deepcopy
from typing import Optional

from simulation import PhysicalState

# SGP4 implementation. Read docs: https://pypi.org/project/sgp4/
from sgp4.api import accelerated, Satrec, WGS72


# Currently set to values of the internation space station as dummy values
# TODO: Make these values coincide with the actual values for Biosat
BIOSAT = Satrec()
BIOSAT.sgp4init(
    WGS72,                # gravity model
    'i',                  # 'a' = old AFSPC mode, 'i' = improved mode
    25544,                # satnum: Satellite number
    25545.69339541,       # epoch: days since 1949 December 31 00:00 UT
    3.8792e-05,           # bstar: drag coefficient (1/earth radii)
    0.0,                  # ndot: ballistic coefficient (radians/minute^2)
    0.0,                  # nddot: mean motion 2nd derivative (radians/minute^3)
    0.0007417,            # ecco: eccentricity
    0.3083420829620822,   # argpo: argument of perigee (radians 0..2pi)
    0.9013560935706996,   # inclo: inclination (radians 0..pi)
    1.4946964807494398,   # mo: mean anomaly (radians 0..2pi)
    0.06763602333248933,  # no_kozai: mean motion (radians/minute)
    3.686137125541276,    # nodeo: R.A. of ascending node (radians 0..2pi)
)


class Satellite:
    state: PhysicalState = PhysicalState()
    J: np.ndarray = np.identity(3) * 0.1        # Inertia matrix
    H: np.ndarray = np.zeros((3, 3))            # Angular momentum
    energy: int = 100
    satrec = BIOSAT

    def __init__(self, state=None, satrec: Optional[Satrec]=None) -> None:
        if state is not None:
            self.state = state
            self.initial_state = state
        else:
            self.initial_state = PhysicalState()

        # SGP4 model
        self.saterec = BIOSAT if satrec is None else satrec


    def reset(self):
        self.state = deepcopy(self.initial_state)


    def load_satellite_parameters(self, state):
        self.initial_state = deepcopy(state)
        self.state = state


if __name__ == '__main__':
    print(accelerated)

