import numpy as np
from copy import deepcopy
from typing import Optional

from simulation import PhysicalState
from Actuators.actuatorSystem import ActuatorSystem

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

    # Magnetic vector in inertial frame
    b0: np.ndarray = np.array([0.0, 0.0, 1.0]) * 65e-6

    satrec = BIOSAT
    energy: int = 100

    def __init__(self,
                 state=None,
                 satrec: Optional[Satrec]=None, 
                 actuator_system: Optional[ActuatorSystem]=None) -> None:
        if state is not None:
            self.state = state
            self.initial_state = state
        else:
            self.initial_state = PhysicalState()

        self.actuator_system = actuator_system if actuator_system is not None else ActuatorSystem()
        # self.actuator_system = ActuatorSystem.init_base_rw_system()

        # SGP4 model
        self.saterec = BIOSAT if satrec is None else satrec


    def reset(self):
        self.state = deepcopy(self.initial_state)
        self.actuator_system.reset()


    def load_satellite_parameters(self, state):
        self.initial_state = deepcopy(state)
        self.state = state

    def get_magnetic_field_vector(self, **kwargs):
        """ Return the magnetic field vector in the BODY frame. Many options
            for how to calculate this, but this implementation will be simple 
            and return a vector of constant magnitude and orientation in the
            inertial frame.
        """
        # NOTE: Magnetic field vector in some inertial frame. Will not change its direction or 
        # magnitude. The magnitude of Earth's magnetic field at its surface ranges from 25 to 65 μT
        B_dir: np.ndarray = np.array([1.0, 1.0, 1.0])
        B: np.ndarray = B_dir * 65e-6 / np.linalg.norm(B_dir)

        return self.state.rot.as_matrix() @ B


if __name__ == '__main__':
    print(accelerated)

