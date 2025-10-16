"""
Actuator System Module

This module defines the ActuatorSystem class, which manages a collection of 
individual actuators (like Reaction Wheels and Magnetorquers) and implements 
the logic for torque distribution and reaction wheel desaturation.
"""

from matplotlib.pyplot import axis
import numpy as np
from Actuators.actuator import Actuator, ActuatorAnimation
from Actuators.reactionwheel import ReactionWheel
from Actuators.magnetorquer import Magnetorquer
from animation import BaseAnimation
from typing import List, MutableSequence, Sequence, Optional, Type

import misc


IDEAL_ACTUATORS = [
    Actuator(axis=np.array([1.0, 0.0, 0.0])),
    Actuator(axis=np.array([0.0, 1.0, 0.0])),
    Actuator(axis=np.array([0.0, 0.0, 1.0]))
]

BASE_RW_CONFIG = [
    ReactionWheel(axis=np.array([1.0, 0.0, 0.0])),
    ReactionWheel(axis=np.array([0.0, 1.0, 0.0])),
    ReactionWheel(axis=np.array([0.0, 0.0, 1.0])),
    ReactionWheel(axis=np.array([1.0, 1.0, 1.0])), 

    Magnetorquer(axis=np.array([1.0, 0.0, 0.0])),
    Magnetorquer(axis=np.array([0.0, 1.0, 0.0])),
    Magnetorquer(axis=np.array([0.0, 0.0, 1.0])),
]


class ActuatorSystem:
    """
    Manages a collection of actuators and handles complex tasks like torque 
    distribution and reaction wheel desaturation.

    The system uses the pseudo-inverse method for allocating commanded torque 
    to individual reaction wheels.

    Attributes
    ----------
    actuators : :class:`typing.Sequence` of :class:`Actuator.Actuator`
        The complete list of all actuators managed by the system. Defaults 
        to ``BASE_RW_CONFIG``.
    magnetorquers : list of :class:`Actuator.Magnetorquer`
        A subset of actuators that are Magnetorquers.
    reaction_wheels : list of :class:`Actuator.ReactionWheel`
        A subset of actuators that are Reaction Wheels.
    A : :class:`numpy.ndarray`
        The configuration matrix $\mathbf{A}$ for the reaction wheels, 
        where each column is the wheel's normalized axis, shape $(3, N_{RW})$.
    A_inv : :class:`numpy.ndarray`
        The pseudo-inverse of the reaction wheel configuration matrix $\mathbf{A}^\dagger$, 
        used for torque distribution, shape $(N_{RW}, 3)$.
    A_mqt : :class:`numpy.ndarray`
        The configuration matrix for the magnetorquers, shape $(3, N_{MQT})$.
    A_mqt_inv : :class:`numpy.ndarray`
        The pseudo-inverse of the magnetorquer configuration matrix, shape $(N_{MQT}, 3)$.
    """

    def __init__(self, actuators: Optional[Sequence[Actuator]]=None) -> None:
        """
        Initializes the Actuator System and calculates the distribution matrices.

        Parameters
        ----------
        actuators : :class:`typing.Sequence` of :class:`Actuator.Actuator`, optional
            The list of actuators to include in the system. If None, 
            ``BASE_RW_CONFIG`` is used.
        """
        self.actuators: Sequence[Actuator] = BASE_RW_CONFIG if actuators is None else actuators

        # Divide the actuators into its sub components in order to better divide labor
        self.magnetorquers: List[Magnetorquer] = [x for x in self.actuators if isinstance(x, Magnetorquer)]
        self.reaction_wheels: List[ReactionWheel] = [x for x in self.actuators if isinstance(x, ReactionWheel)]

        # NOTE: Only uses reaction wheels for pointing control. The magnetorquers are for desaturation
        self.A = np.array([x.axis for x in self.reaction_wheels]).T
        self.A_inv = np.linalg.pinv(self.A)

        self.A_mqt = np.array([x.axis for x in self.magnetorquers]).T
        self.A_mqt_inv = np.linalg.pinv(self.A_mqt)


    # TODO: Implement desaturation of reaction wheels

    @classmethod
    def init_base_rw_system(cls):
        """
        Initializes an ActuatorSystem configured only with a basic set of 
        four Reaction Wheels (three orthogonal, one tilted).

        Returns
        -------
        :class:`ActuatorSystem`
            A new ActuatorSystem instance with only reaction wheels.
        """
        base_rw = [
            ReactionWheel(axis=np.array([1.0, 0.0, 0.0])),
            ReactionWheel(axis=np.array([0.0, 1.0, 0.0])),
            ReactionWheel(axis=np.array([0.0, 0.0, 1.0])),
            ReactionWheel(axis=np.array([1.0, 1.0, 1.0]))
        ]
        return cls(actuators=base_rw)

    def angular_momentum_wheels(self):
        """
        Calculates the total angular momentum vector $\mathbf{h}_w$ of all 
        reaction wheels in the body frame.

        The total angular momentum is the sum of each wheel's angular momentum 
        $\mathbf{h}_i = J_i \mathbf{a}_i \omega_i$.

        Returns
        -------
        :class:`numpy.ndarray`
            The total angular momentum $\mathbf{h}_w$ in the body frame, shape (3,).
        """
        hw = np.zeros(3)
        for wheel in self.reaction_wheels:
            hw += wheel.J @ (wheel.axis * wheel.w)
        return hw

    def ref_angular_momentum_wheels(self, w_ref: float, rad: bool=True) -> np.ndarray: 
        """
        Calculates the total angular momentum vector $\mathbf{h}_{ref}$ if all 
        reaction wheels were spinning at a constant reference angular velocity $\omega_{ref}$.

        Parameters
        ----------
        w_ref : float
            The reference angular velocity magnitude.
        rad : bool, optional
            If True (default), `w_ref` is in rad/s. If False, it is converted 
            from RPM to rad/s.

        Returns
        -------
        :class:`numpy.ndarray`
            The reference angular momentum $\mathbf{h}_{ref}$ in the body frame, shape (3,).
        """
        # Convert to rad/s
        if rad == False:
            w_ref = w_ref * np.pi / 30

        hw = np.zeros(3)
        for wheel in self.reaction_wheels:
            hw += wheel.J @ (wheel.axis * w_ref)
        return hw

    def distribute_torque_rw(self, tau_c: np.ndarray) -> np.ndarray:
        """
        Distributes the commanded torque vector $\mathbf{\tau}_c$ over the 
        reaction wheels to generate the resulting torque $\mathbf{L}_w$.

        The torque command $\mathbf{u}_{wheels}$ for each wheel is determined 
        using the pseudo-inverse: $\mathbf{u}_{wheels} = \mathbf{A}^\dagger \mathbf{\tau}_c$.

        Parameters
        ----------
        tau_c : :class:`numpy.ndarray`
            The commanded torque vector in the body frame, shape (3,).

        Returns
        -------
        :class:`numpy.ndarray`
            The total applied torque $\mathbf{L}_w$ generated by the reaction wheels 
            in the body frame, shape (3,).
        """
        # NOTE: Reaction wheel torque for pointing control. Torque
        # is distributed by the pseudo inverse matrix A_inv
        L = np.zeros_like(tau_c)
        u_wheels = self.A_inv @ tau_c   # Torque for each wheel
        for i, ac in enumerate(self.reaction_wheels):
            L += ac.axis * ac.apply_torque(u_wheels[i])
        return L

    def distribute_torque_mqt(self, tau_m: np.ndarray, b: np.ndarray) -> np.ndarray:
        """
        Calculates the required dipole moments $\mathbf{m}$ to produce a 
        commanded magnetic torque $\mathbf{\tau}_m$ and distributes it across 
        the magnetorquers.

        Note: The torque $\mathbf{\tau}$ produced by a magnetorquer depends on 
        the magnetic field $\mathbf{B}$ as $\mathbf{\tau} = \mathbf{m} \times \mathbf{B}$.

        Parameters
        ----------
        tau_m : :class:`numpy.ndarray`
            The commanded magnetic torque vector $\mathbf{\tau}_m$ in the body frame, shape (3,).
        b : :class:`numpy.ndarray`
            The magnetic field vector $\mathbf{B}$ in the body frame, shape (3,).

        Returns
        -------
        :class:`numpy.ndarray`
            The vector of actual applied dipole moments $\mathbf{m}$ (before 
            the cross product with B), shape (N_MQT,).
        """
        L = np.zeros_like(tau_m)
        tau_A = self.A_mqt_inv @ tau_m
        for i, mqt in enumerate(self.magnetorquers):
            L[i] = mqt.apply_torque(tau_A[i])

            # Log data for displaying
            t_i: np.ndarray = np.cross(b, L[i] * mqt.axis)
            ref_i: np.ndarray = np.cross(b, tau_A)
            mqt.log_data("torque", float(np.linalg.norm(t_i, 2)))
            mqt.log_data("reference torque", float(np.linalg.norm(ref_i, 2)))
        return L

    def desaturate(self, b0: np.ndarray, R: np.ndarray, w_ref: float = 1.0, kp: float=1.0):
        """
        Calculates the required magnetic torque $\mathbf{\tau}_m$ in the body frame 
        to desaturate the reaction wheels using the static input allocation method.

        The method implements Equation (25) from the Tregouet et al. paper 
        (Link to paper: https://hal.science/hal-01760720/document).

        $$
            \mathbf{\tau}_m = -\frac{(\mathbf{R}\mathbf{B}_0)^{\times}}{|\mathbf{B}_0|^{2}}k_p(\mathbf{h}_\omega - \mathbf{h}_{ref})
        $$

        Parameters
        ----------
        b0 : :class:`numpy.ndarray`
            The magnetic field vector $\mathbf{B}_0$ in the **inertial frame**, shape (3,).
        R : :class:`numpy.ndarray`
            The rotation matrix from the inertial frame to the **body frame**, $\mathbf{R}$, 
            shape (3, 3).
        w_ref : float, optional
            The target reference angular speed for the wheels $\omega_{ref}$ (rad/s). Defaults to 1.0.
        kp : float, optional
            The proportional control gain for desaturation. Defaults to 1.0.

        Returns
        -------
        :class:`numpy.ndarray`
            The required magnetic torque $\mathbf{\tau}_m$ in the body frame, shape (3,).

        Raises
        ------
        AssertionError
            If `b0` is not R^3, `R` is not 3x3, or `kp` is not positive.
        """

        # WARN: Magnetic field vector needs to be expressed in inertial frame, and R is from inertial to body
        assert len(b0) == 3 and np.ndim(b0) == 1, "The magnetic field vector needs to be R^3"
        assert R.shape == (3,3), "The rotation matrix needs to be R^{3x3}"
        assert kp > 0.0, "The gain kp needs to be positive"

        h_ref = self.ref_angular_momentum_wheels(w_ref)

        # BODY frame implementation
        prelim_feedback = -misc.skew(R.T @ b0) / (np.linalg.norm(b0)**2)
        diff = kp * (self.angular_momentum_wheels() - h_ref)

        # Inertial frame implementation
        # prelim_feedback = -misc.skew(b0) / (np.linalg.norm(b0)**2)
        # diff = kp * (self.angular_momentum_wheels() - R.T @ h_ref)

        tau_m = prelim_feedback @ diff

        return tau_m


    def apply_torque(self, u: np.ndarray, **kwargs) -> np.ndarray:
        """
        Calculates the net external torque $\mathbf{\tau}_{ext}$ applied to the 
        satellite body by the complete actuation system (wheels and magnetorquers).

        The method implements the total torque command $\mathbf{\tau}_{w+m}$ which 
        is distributed to the wheels and magnetorquers, and then returns the 
        actual torque applied to the body, accounting for gyroscopic effects.

        The final applied torque $\mathbf{L}$ on the satellite body is:
        $\mathbf{L} = -\dot{\mathbf{h}}_w + \mathbf{\tau}_m - \mathbf{\omega} \times \mathbf{h}_w$
        where $\mathbf{\tau}_{rw} = \dot{\mathbf{h}}_w$.

        The torque commanded to the Reaction Wheels ($\mathbf{\tau}_{w}$) is:
        $\mathbf{\tau}_w = -\mathbf{\omega} \times \mathbf{h}_w + \mathbf{T}_m - \mathbf{u}$
        where $\mathbf{u}$ is the commanded pointing torque, $\mathbf{T}_m$ is the 
        torque from the magnetorquers.

        Parameters
        ----------
        u : :class:`numpy.ndarray`
            The commanded pointing torque vector from the attitude controller, $\mathbf{u}$, shape (3,).
        **kwargs : dict
            Required parameters for desaturation and gyroscopic effects:
            
            * **b0** (:class:`numpy.ndarray`): Magnetic field in the **inertial** frame (default: [0, 0, 1]).
            * **R** (:class:`numpy.ndarray`): Rotation matrix (Inertial to Body) (default: Identity).
            * **w_ref** (float): Reference wheel speed for desaturation (default: 1.0 rad/s).
            * **kp** (float): Proportional gain for desaturation (default: 1.0).
            * **w** (:class:`numpy.ndarray`): Satellite angular velocity in the body frame, $\mathbf{\omega}$ (default: zeros).

        Returns
        -------
        :class:`numpy.ndarray`
            The net torque applied to the satellite body $\mathbf{\tau}_{ext}$ 
            in the body frame, shape (3,).
        """

        # Get the necessary parameters for desaturation
        b0 = kwargs.get("b0", np.array([0.0, 0.0, 1.0]))
        omega = kwargs.get("w", np.zeros(3))
        w_ref = kwargs.get("w_ref", 1.0)
        R = kwargs.get("R", np.eye(3))
        kp = kwargs.get("kp", 1.0)

        # b0 = -R @ b

        # Calculate the magnetorquer actuation
        tau_m = self.desaturate(b0, R, w_ref, kp)
        t_m = self.distribute_torque_mqt(tau_m, b=R @ b0)
        T_m = -np.cross(R @ b0, t_m)

        # Calculate the reaction wheel actuation
        hw = self.angular_momentum_wheels()
        tau_w = -misc.skew(omega) @ hw + T_m - u
        L_w = self.distribute_torque_rw(tau_w)

        return -L_w + T_m - misc.skew(omega) @ hw

    def reset(self):
        """
        Resets the state of all individual actuators within the system.

        Currently, this sets the speed of all reaction wheels to zero. Any other 
        reset logic for other actuator types must be implemented here.
        """
        for wheel in self.reaction_wheels:
            wheel.w = 0.0

    def __str__(self) -> str:
        s = "Actuator System: \n"
        for i, ac in enumerate(self.actuators):
            if isinstance(ac, ReactionWheel):
                s += f"RW: Max current: {ac.max_I} \t"
            if isinstance(ac, Magnetorquer):
                s += f"MQT: "
            # s += f"{ac.axis}\t | |{self.A[i, 0]} - {self.A[i, 1]} - {self.A[i, 2]} |\n"
            s += f"{ac.axis}\n"

        s += f"A: {self.A}\n"
        s += f"{[x.axis for x in self.actuators]}"
        return s


def test_torque():
    # np.set_printoptions(precision=3)
    np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
    a = ActuatorSystem()
    b = ActuatorSystem.init_base_rw_system()
    for rw in a.actuators:
        if isinstance(rw, ReactionWheel):
            rw.max_I = np.inf

    for rw in b.actuators:
        if isinstance(rw, ReactionWheel):
            rw.max_I = np.inf

    print(a.A.shape)
    print(a)
    print(b)

    taus = [
        np.array([1.0, 3.0, 1.0]),
        np.array([4.0, 4.0, 1.0]),
        np.array([8.0, 2.0, 2.0]),
        np.array([5.0, 3.0, 2.0]),
        np.array([1.0, 1.1, 4.0]),
        np.array([1.0, 1.0, 1.0]),
    ]

    print()
    for i, t in enumerate(taus):
        L = a.apply_torque(t)
        print(f"Test case: reference: {t} | applied: {L}")

    print()
    for i, t in enumerate(taus):
        L = b.apply_torque(t)
        print(f"Test case: reference: {t} | applied: {L}")


if __name__ == '__main__':
    test_torque()
