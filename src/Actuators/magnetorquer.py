import numpy as np
from Actuators.actuator import Actuator
import matplotlib.pyplot as plt
from typing import Optional, Union


class Magnetorquer(Actuator):
    # TODO: Calculate the magnetic field strength based on the position of the satellite
    # TODO: Calculate torque based on the strength of the magnetic field
    # TODO: Calculate energy output from current etc.
    # TODO: Calculate torque from current available

    name: str = "Magnetorquer"

    # No use for these at this moment
    maxTorque: float = 1.0
    scalingFactor: float = 1.0

    k: float = 1.0                  # Magnetic dipole scalar. k = n A => m = k * I
    max_I: float = 1.0              # Max available current

    # NOTE: Magnetic field vector in some inertial frame. Will not change its direction or 
    # magnitude. The magnitude of Earth's magnetic field at its surface ranges from 25 to 65 μT
    B: np.ndarray = np.array([1.0, 0.0, 0.0]) * 65e-6

    def __init__(self, axis: Optional[np.ndarray]=None) -> None:
        super().__init__(axis)

        # NOTE: Log some dummy data for the actuator to display at start
        self.log_data("torque", 0)
        self.log_data("reference torque", 0)
        self.log_data("dipole", 0)
        self.log_data("reference", 0)
        self.log_data("I", 0)

    def torque(self,
               B: np.ndarray,
               m: Optional[Union[float, np.ndarray]]=None):
        """ Calculate the torque based on the magnetic dipole m and the magnetic field:
            strength B. According to: 

                tau = m \times B

            Assumes: 
                - m expressed in BODY
                - B expressed in BODY frame

            returns the results torque vector expressed in the BODY frame
        """
        m = m if type(m) is np.ndarray else m
        m = self.m * self.axis if m is None else m
        m = self.axis * m if type(m) is float else m

        return np.cross(m, B)

    def apply_torque(self, tau, dt: float = 0.1, **kwargs):
        """ The torque provided from the magnettorquer is essentially instant, 
            and thus does not need any model of the transient response. 

            tau is the requested torque from the actuator system. tau is a float,
            where it is expected to provide a torque magnitude that matches tau along
            the axis of the actuator. The torque depends on the magnetic field vector,
            B in the body frame, which is why it is passed in **kwargs. The torque
            calculated is a vector that IS NOT THE SAME AS THE AXIS FOR THE ACTUATOR.
            The actuator system needs to account for this, and is not the job of individual
            actuators. The only thing that the actuator system is requesting is an idealised
            actuation from the actuator, for a mqt, that would be the magnetic dipole, as 
            oppsed to the actual torque for the mqt.
        """
        # WARN: BYPASSING FUNCTION FOR TESTING. REMOVE LATER!!!
        # return tau

        # NOTE: Calculate the current where the provided magnetic dipole matches the commanded
        # dipole tau
        assert isinstance(tau, float) or isinstance(tau, int) or isinstance(tau, np.floating) \
        , f"tau needs to be a float or integer. not {type(tau)}"

        I = tau / self.k
        I = self.saturate(value=I, minimum=-self.max_I, maximum=self.max_I)

        L =  self.k * I

        # Remember to log tha data
        self.log_data("dipole", L)
        self.log_data("reference", tau)
        self.log_data("I", I)

        return L


def testTorque(mqt:Magnetorquer):
    dt = 0.001
    T = np.arange(0.0, 100.0, dt)
    L = np.zeros_like(T)
    LC = np.zeros_like(T)

    for i, t in enumerate(T):
        # LC[i] = 0.6 * mqt.maxTorque
        LC[i] = 1.2 * mqt.maxTorque * np.sin(t)
        L[i] = np.linalg.norm(mqt.apply_torque(LC[i], dt), 2)

    fig_L = plt.figure(2)
    plt.title("Torque plot")
    plt.plot(T, L, 'b')
    plt.plot(T, LC, '--g')
    plt.plot([T[0], T[-1]], [-mqt.maxTorque, -mqt.maxTorque], '--r')
    plt.plot([T[0], T[-1]], [mqt.maxTorque, mqt.maxTorque], '--r')
    plt.grid()

    plt.show()

if __name__ == "__main__":
    mtq = Magnetorquer()
    testTorque(mtq)

