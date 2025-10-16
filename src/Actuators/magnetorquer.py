import numpy as np
from Actuators.actuator import Actuator
import matplotlib.pyplot as plt
from typing import Optional, Union


class Magnetorquer(Actuator):
    """
    Represents a Magnetorquer (or magnetic torque rod) actuator.

    Magnetorquers generate torque by creating a magnetic dipole moment $\mathbf{m}$ 
    that interacts with the local magnetic field $\mathbf{B}$ via the cross product 
    $\mathbf{\tau} = \mathbf{m} \times \mathbf{B}$.

    Inherits from :class:`Actuators.actuator.Actuator`.

    Attributes
    ----------
    name : str
        The name of the actuator: "Magnetorquer".
    maxTorque : float
        (Currently unused) Maximum theoretical torque magnitude. Defaults to 1.0 N$\cdot$m.
    scalingFactor : float
        (Currently unused) Generic scaling factor. Defaults to 1.0.
    k : float
        Magnetic dipole scalar $k = n A$, where $n$ is the number of turns and $A$ 
        is the coil area. This relates current $I$ to dipole $m$: $\mathbf{m} = k I \mathbf{a}$, 
        where $\mathbf{a}$ is the actuator axis. Defaults to 1.0.
    max_I : float
        Maximum available current (in Amperes). Used for saturation. Defaults to 1.0 A.
    B : :class:`numpy.ndarray`
        A placeholder for the nominal or initial magnetic field vector (inertial frame), 
        set to $65 \mu T$ along the x-axis. This attribute is typically **overridden** by the magnetic field provided during a simulation step.
    """
    # TODO: Calculate energy output from current etc.

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
        """
        Initializes the Magnetorquer.

        Parameters
        ----------
        axis : :class:`numpy.ndarray`, optional
            The direction vector of the magnetorquer's coil axis in the body frame. 
            Inherited from :class:`Actuators.actuator.Actuator`.
        """
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
        """
        Calculates the torque vector $\mathbf{\tau}$ based on the magnetic dipole 
        moment $\mathbf{m}$ and the magnetic field strength $\mathbf{B}$.

        $$
            \mathbf{\tau} = \mathbf{m} \times \mathbf{B}
        $$

        It is assumed that both $\mathbf{m}$ and $\mathbf{B}$ are expressed in the 
        **body frame**.

        Parameters
        ----------
        B : :class:`numpy.ndarray`
            The magnetic field vector $\mathbf{B}$ in the body frame, shape (3,).
        m : float or :class:`numpy.ndarray`, optional
            The magnetic dipole moment. If a float, it is assumed to be the 
            scalar magnitude $m$, and the resulting dipole is $\mathbf{m} = m \cdot \mathbf{axis}$. 
            If None, the function assumes a pre-existing dipole magnitude $m_{current}$ 
            is used, resulting in $\mathbf{m} = m_{current} \cdot \mathbf{axis}$.

        Returns
        -------
        :class:`numpy.ndarray`
            The resulting torque vector $\mathbf{\tau}$ in the body frame, shape (3,).
        """
        m = m if type(m) is np.ndarray else m
        m = self.m * self.axis if m is None else m
        m = self.axis * m if type(m) is float else m

        return np.cross(m, B)

    def apply_torque(self, tau, dt: float = 0.1, **kwargs):
        """
        Calculates the actual magnetic dipole moment $\mathbf{m}$ commanded by the 
        actuator system, subject to current saturation.

        Unlike other actuators (like reaction wheels), the input $\tau$ is not 
        the desired *torque* magnitude, but the desired **magnetic dipole magnitude** $m$ 
        along the magnetorquer's axis $\mathbf{a}$ required by the actuation system 
        to produce the necessary torque $\mathbf{\tau}$.

        The commanded current $I$ is calculated as $I = m / k$.

        Parameters
        ----------
        tau : float
            The scalar magnitude of the **commanded magnetic dipole moment** $m_{c}$ 
            along the actuator's axis, $m_{c} = \mathbf{m}_c \cdot \mathbf{a}$.
        dt : float, optional
            The time step duration in seconds (unused for instantaneous magnetorquers). 
            Defaults to 0.1 s.
        **kwargs : dict
            Additional parameters (e.g., magnetic field $\mathbf{B}$ which is needed 
            for external torque calculation, but not for this method).

        Returns
        -------
        float
            The scalar magnitude of the **actual magnetic dipole moment** $m$ 
            generated by the magnetorquer, after saturation.
        """

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
    """
    Test function to simulate a commanded dipole moment and observe saturation.
    """
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

