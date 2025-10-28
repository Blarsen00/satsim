import numpy as np
from Actuators.actuator import Actuator
import matplotlib.pyplot as plt
from typing import Optional, Union


class Magnetorquer(Actuator):
    """
    Represents a Magnetorquer (or magnetic torque rod) actuator.

    Magnetorquers generate torque by creating a magnetic dipole moment :math:`\\mathbf{m}` 
    that interacts with the local magnetic field :math:`\\mathbf{B}` via the cross product 
    :math:`\\mathbf{\\tau} = \\mathbf{m} \\times \\mathbf{B}`.

    Inherits from :class:`Actuators.actuator.Actuator`.

    Attributes
    ----------
    name : str
        The name of the actuator: "Magnetorquer".
    maxTorque : float
        (Currently unused) Maximum theoretical torque magnitude. Defaults to 1.0 N:math:`\\cdot`m.
    scalingFactor : float
        (Currently unused) Generic scaling factor. Defaults to 1.0.
    k : float
        Magnetic dipole scalar :math:`k = n A`, where :math:`n` is the number of turns and :math:`A` 
        is the coil area. This relates current :math:`I` to dipole :math:`m`: :math:`\\mathbf{m} = k I \\mathbf{a}`, 
        where :math:`\\mathbf{a}` is the actuator axis. Defaults to 1.0.
    max_I : float
        Maximum available current (in Amperes). Used for saturation. Defaults to 1.0 A.
    B : :class:`numpy.ndarray`
        A placeholder for the nominal or initial magnetic field vector (inertial frame), 
        set to :math:`65 \\mu T` along the x-axis. This attribute is typically **overridden** by the magnetic field provided during a simulation step.
    """
    # TODO: Calculate energy output from current etc.

    name: str = "Magnetorquer"

    # No use for these at this moment
    # maxTorque: float = 1.0
    # scalingFactor: float = 1.0

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
        self.k: float = 1.0              # Magnetic dipole scalar. k = n A => m = k * I
        self.max_I: float = 1.0           # Max available current
        self.maxTorque: float = 1.0       # Added as it was missing from the attributes/test
        self._params = ["axis", "k", "max_I"]

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
        Calculates the torque vector :math:`\\mathbf{\\tau}` based on the magnetic dipole 
        moment :math:`\\mathbf{m}` and the magnetic field strength :math:`\\mathbf{B}`.

        .. math::
            \\mathbf{\\tau} = \\mathbf{m} \\times \\mathbf{B}

        It is assumed that both :math:`\\mathbf{m}` and :math:`\\mathbf{B}` are expressed in the 
        **body frame**.

        Parameters
        ----------
        B : :class:`numpy.ndarray`
            The magnetic field vector :math:`\\mathbf{B}` in the body frame, shape (3,).
        m : float or :class:`numpy.ndarray`, optional
            The magnetic dipole moment. If a float, it is assumed to be the 
            scalar magnitude :math:`m`, and the resulting dipole is :math:`\\mathbf{m} = m \\cdot \\mathbf{axis}`. 
            If None, the function assumes a pre-existing dipole magnitude :math:`m_{current}` 
            is used, resulting in :math:`\\mathbf{m} = m_{current} \\cdot \\mathbf{axis}`.

        Returns
        -------
        :class:`numpy.ndarray`
            The resulting torque vector :math:`\\mathbf{\\tau}` in the body frame, shape (3,).
        """
        if m is None:
            m_vector = 0.0 * self.axis 

        elif isinstance(m, (float, np.floating, int)):
            m_vector = self.axis * m

        else:
            m_vector = m

        return np.cross(m_vector, B)

    def apply_torque(self, tau, dt: float = 0.1, **kwargs):
        """
        Calculates the actual magnetic dipole moment :math:`\\mathbf{m}` commanded by the 
        actuator system, subject to current saturation.

        Unlike other actuators (like reaction wheels), the input :math:`\\tau` is not 
        the desired *torque* magnitude, but the desired **magnetic dipole magnitude** :math:`m` 
        along the magnetorquer's axis :math:`\\mathbf{a}` required by the actuation system 
        to produce the necessary torque :math:`\\mathbf{\\tau}`.

        The commanded current :math:`I` is calculated as :math:`I = m / k`.

        Parameters
        ----------
        tau : float
            The scalar magnitude of the **commanded magnetic dipole moment** :math:`m_{c}` 
            along the actuator's axis, :math:`m_{c} = \\mathbf{m}_c \\cdot \\mathbf{a}`.
        dt : float, optional
            The time step duration in seconds (unused for instantaneous magnetorquers). 
            Defaults to 0.1 s.
        **kwargs : dict
            Additional parameters (e.g., magnetic field :math:`\\mathbf{B}` which is needed 
            for external torque calculation, but not for this method).

        Returns
        -------
        float
            The scalar magnitude of the **actual magnetic dipole moment** :math:`m` 
            generated by the magnetorquer, after saturation.
        """

        # NOTE: Calculate the current where the provided magnetic dipole matches the commanded
        # dipole tau
        assert isinstance(tau, (float, int, np.floating)) \
        , f"tau needs to be a float or integer. not {type(tau)}"

        I = tau / self.k
        I = self.saturate(value=I, minimum=-self.max_I, maximum=self.max_I)

        L = self.k * I # L is the actual dipole magnitude m

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
        # The test case is incorrectly using maxTorque, it should use max_I or a scaled value of max_I
        # If we assume maxTorque is just a placeholder and the saturation is controlled by max_I, 
        # let's replace maxTorque with a scaled version of max_I (which translates to max dipole L)
        # Assuming k=1 and max_I=1, the max dipole is 1.0. 
        # The test function is incorrectly designed as apply_torque returns L (dipole magnitude)
        # while the original code in the test was comparing it to maxTorque.
        # To make the test run as intended (plotting saturation), we'll assume maxTorque 
        # in the test should represent the maximum possible dipole moment (max_L)
        
        max_L = mqt.k * mqt.max_I # Max dipole moment
        
        # LC[i] = 0.6 * max_L
        LC[i] = 1.2 * max_L * np.sin(t)
        # apply_torque returns L (scalar dipole magnitude) not torque vector, so np.linalg.norm is unnecessary and incorrect here.
        # It's only incorrect if the original intention was to plot torque magnitude.
        # Given the original code: L[i] = np.linalg.norm(mqt.apply_torque(LC[i], dt), 2)
        # apply_torque returns L (float). np.linalg.norm(float, 2) is a float. It seems L[i] is just being set to the return value L.
        L[i] = mqt.apply_torque(LC[i], dt)

    fig_L = plt.figure(2)
    plt.title("Dipole Moment Plot (Saturation Test)")
    # Replacing maxTorque with the calculated max dipole moment for correct plot limits
    max_L = mqt.k * mqt.max_I
    
    plt.plot(T, L, 'b', label="Actual Dipole m (scalar)")
    plt.plot(T, LC, '--g', label="Commanded Dipole $m_c$ (scalar)")
    plt.plot([T[0], T[-1]], [-max_L, -max_L], '--r', label="Max Dipole Limit")
    plt.plot([T[0], T[-1]], [max_L, max_L], '--r')
    plt.xlabel("Time (s)")
    plt.ylabel("Dipole Moment (A$\cdot$m$^2$)")
    plt.legend()
    plt.grid()

    plt.show()

if __name__ == "__main__":
    mtq = Magnetorquer()
    testTorque(mtq)
