import numpy as np
from actuator import Actuator
import matplotlib.pyplot as plt
from typing import Optional, Union


class Magnetorquer(Actuator):
    # TODO: Calculate the magnetic field strength based on the position of the satellite
    # TODO: Calculate torque based on the strength of the magnetic field
    # TODO: Calculate energy output from current etc.
    # TODO: Calculate torque from current available

    # No use for these at this moment
    maxTorque: float = 1.0
    scalingFactor: float = 1.0

    m: float = 1.0                  # Magnetic dipole scalar. m = n I A
    max_I: float = 1.0              # Max available current

    def __init__(self, axis: Optional[np.ndarray]=None) -> None:
        super().__init__(axis)

    @classmethod
    def torque(cls, B: np.ndarray, m: Optional[Union[float, np.ndarray]]=None):
        """ Calculate the torque based on the magnetic dipole m and the magnetic field 
            strength B. According to: 

                tau = m \cross B

            Assuming that both vectors m and B are in the same reference frame.
        """
        m = m if type(m) is np.ndarray else m
        m = cls.m * cls.axis if m is None else m
        m = cls.axis * m if type(m) is float else m

        return np.cross(m, B)

    def apply_torque(self, L, dt):
        """ The torque provided from the magnettorquer is essentially instant, 
            and thus does not need any model of the transient response, and 
            just the statuarted output does just fine for now.
        """
        return Actuator.saturate(L, self.maxTorque, -self.maxTorque)



def testTorque(mqt:Magnetorquer):
    dt = 0.001
    T = np.arange(0.0, 100.0, dt)
    L = np.zeros_like(T)
    LC = np.zeros_like(T)

    for i, t in enumerate(T):
        # LC[i] = 0.6 * mqt.maxTorque
        LC[i] = 1.2 * mqt.maxTorque * np.sin(t)
        L[i] = mqt.apply_torque(LC[i], dt)

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

