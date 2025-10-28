import numpy as np
from copy import deepcopy
from typing import Optional

from simulation import PhysicalState
from Actuators.actuatorSystem import ActuatorSystem

# SGP4 implementation. Read docs: https://pypi.org/project/sgp4/
# from sgp4.api import accelerated, Satrec, WGS72


# TODO: Make these values coincide with the actual values for Biosat
# Currently set to values of the international space station as dummy values
# BIOSAT = Satrec()
# BIOSAT.sgp4init(
#     WGS72,                         # gravity model
#     'i',                           # 'a' = old AFSPC mode, 'i' = improved mode
#     25544,                         # satnum: Satellite number
#     25545.69339541,                # epoch: days since 1949 December 31 00:00 UT
#     3.8792e-05,                    # bstar: drag coefficient (1/earth radii)
#     0.0,                           # ndot: ballistic coefficient (radians/minute^2)
#     0.0,                           # nddot: mean motion 2nd derivative (radians/minute^3)
#     0.0007417,                     # ecco: eccentricity
#     0.3083420829620822,            # argpo: argument of perigee (radians 0..2pi)
#     0.9013560935706996,            # inclo: inclination (radians 0..pi)
#     1.4946964807494398,            # mo: mean anomaly (radians 0..2pi)
#     0.06763602333248933,           # no_kozai: mean motion (radians/minute)
#     3.686137125541276,             # nodeo: R.A. of ascending node (radians 0..2pi)
# )


class Satellite:
    """
    Represents a satellite object for simulation, containing its physical state,
    inertia properties, and related systems.

    Attributes
    ----------
    state : :class:`simulation.PhysicalState`
        The current physical state (attitude, angular velocity, etc.) of the satellite.
        Defaults to an identity state.
    initial_state : :class:`simulation.PhysicalState`
        The state of the satellite when the simulation was started or last reset.
    J : :class:`numpy.ndarray`
        The inertia matrix :math:`\\mathbf{J}` of the satellite, shape (3, 3).
        Defaults to :math:`0.1 \\times \\mathbf{I}`.
    H : :class:`numpy.ndarray`
        The total angular momentum vector :math:`\\mathbf{H}` (though initialized as a matrix,
        it's typically a vector in physics context).
        Defaults to a :math:`3 \\times 3` zero array.
    b0 : :class:`numpy.ndarray`
        The magnetic field vector :math:`\\mathbf{B}_0` in the **inertial frame**, used
        for simple magnetic torque calculations. Defaults to :math:`[0, 0, 1] \\times 65\\mu\\text{T}`.
    satrec : :class:`sgp4.api.Satrec`
        The SGP4 satellite record object, used for orbit propagation.
        Defaults to the global ``BIOSAT`` record.
    actuator_system : :class:`Actuators.actuatorSystem.ActuatorSystem`
        The system managing all actuators (e.g., magnetorquers, reaction wheels).
    energy : int
        A placeholder for satellite energy or power consumption status. Defaults to 100.
    """
    state: PhysicalState = PhysicalState()
    J: np.ndarray = np.identity(3) * 0.1          # Inertia matrix
    H: np.ndarray = np.zeros((3, 3))             # Angular momentum

    # Magnetic vector in inertial frame
    b0: np.ndarray = np.array([0.0, 0.0, 1.0]) * 65e-6

    # TODO: Implement some functionality to asses the orbit of the satellite
    # satrec = BIOSAT

    # TODO: Implement some functionality to asses the energy expenditure
    energy: int = 100

    def __init__(self,
                 state: Optional[PhysicalState]=None,
                 # satrec: Optional[Satrec]=None,
                 actuator_system: Optional[ActuatorSystem]=None) -> None:
        """
        Initializes the Satellite object.

        Parameters
        ----------
        state : :class:`simulation.PhysicalState`, optional
            The initial physical state. If None, a default :class:`PhysicalState` is used.
        satrec : :class:`sgp4.api.Satrec`, optional
            The SGP4 satellite record. If None, the global ``BIOSAT`` record is used.
        actuator_system : :class:`Actuators.actuatorSystem.ActuatorSystem`, optional
            The actuator system. If None, a new :class:`ActuatorSystem` is created.
        """
        if state is not None:
            self.state = state
            self.initial_state = deepcopy(state)
        else:
            self.initial_state = PhysicalState()
            self.state = PhysicalState() # Ensure state is also initialized if state=None

        self.actuator_system = actuator_system if actuator_system is not None else ActuatorSystem()

        # SGP4 model
        # self.saterec = BIOSAT if satrec is None else satrec


    def reset(self):
        """
        Resets the satellite's current state to its initial state
        (:attr:`initial_state`) and resets the actuator system.
        """
        self.state = deepcopy(self.initial_state)
        self.actuator_system.reset()


    def load_satellite_parameters(self, state: PhysicalState):
        """
        Sets both the initial and current physical state of the satellite.

        Parameters
        ----------
        state : :class:`simulation.PhysicalState`
            The new physical state to load.
        """
        self.initial_state = deepcopy(state)
        self.state = state

    def get_magnetic_field_vector(self, **kwargs) -> np.ndarray:
        # TODO: Make use of the World Magnetic Model (WMM) in order 
        # to use the position of the satellite to get the magnetic field
        # vector. See https://pypi.org/project/pygeomag/
        """
        Returns the magnetic field vector :math:`\\mathbf{B}` in the **body frame**.

        This implementation uses a simple model where the magnetic field
        vector :math:`\\mathbf{B}_{inertial}` is constant in the inertial frame.
        The body-frame vector is calculated via rotation:
        :math:`\\mathbf{B}_{body} = \\mathbf{R} \\mathbf{B}_{inertial}`

        Parameters
        ----------
        **kwargs : dict
            Currently unused, but allows for expansion to more complex models
            that might require time, position, etc.

        Returns
        -------
        :class:`numpy.ndarray`
            The magnetic field vector :math:`\\mathbf{B}` in the body frame, shape (3,).
        """
        # NOTE: Magnetic field vector in some inertial frame. Will not change its direction or
        # magnitude. The magnitude of Earth's magnetic field at its surface ranges from 25 to 65 μT

        # Define an arbitrary inertial B-field vector for this simplified model
        B_dir: np.ndarray = np.array([1.0, 1.0, 1.0])
        B_inertial: np.ndarray = B_dir * 65e-6 / np.linalg.norm(B_dir)

        # Transform B_inertial to the body frame using the current attitude rotation matrix R
        return self.state.rot.as_matrix() @ B_inertial


if __name__ == '__main__':
    print(accelerated)
