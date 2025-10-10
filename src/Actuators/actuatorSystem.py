from matplotlib.pyplot import axis
import numpy as np
from Actuators.actuator import Actuator, ActuatorAnimation
from Actuators.reactionwheel import ReactionWheel
from Actuators.magnetorquer import Magnetorquer
from animation import BaseAnimation
from typing import List, MutableSequence, Sequence, Optional, Type, Self

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
    # actuators: Sequence[Actuator] = BASE_RW_CONFIG
    # animations: List[ActuatorAnimation]

    def __init__(self, actuators: Optional[Sequence[Actuator]]=None) -> None:
        # print([x.axis for x in BASE_RW_CONFIG])
        self.actuators: Sequence[Actuator] = BASE_RW_CONFIG if actuators is None else actuators
        # self.actuators: MutableSequence[Actuator] = BASE_RW_CONFIG if actuators is None else actuators
        # self.animations = [ActuatorAnimation(ac) for ac in self.actuators]

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
    def init_base_rw_system(cls) -> Self:
        base_rw = [
            ReactionWheel(axis=np.array([1.0, 0.0, 0.0])),
            ReactionWheel(axis=np.array([0.0, 1.0, 0.0])),
            ReactionWheel(axis=np.array([0.0, 0.0, 1.0])),
            ReactionWheel(axis=np.array([1.0, 1.0, 1.0]))
        ]
        return cls(actuators=base_rw)

    def angular_momentum_wheels(self):
        """ Calculate the total angular momentum of the reaction wheels.
        """
        hw = np.zeros(3)
        for wheel in self.reaction_wheels:
            hw += wheel.J @ (wheel.axis * wheel.w)
        return hw

    def ref_angular_momentum_wheels(self, w_ref: float, rad: bool=True) -> np.ndarray: 
        """ Return the total angular momentum from the wheels if they are
            each rotating at the w_ref angular velocity. 

            rad=True => w_ref   (rad / s)
            rad=False => w_ref  (rpm)

            Returns:
                h_w in R^{3}
        """
        # Convert to rad/s
        if rad == False:
            w_ref = w_ref * np.pi / 30

        hw = np.zeros(3)
        for wheel in self.reaction_wheels:
            hw += wheel.J @ (wheel.axis * w_ref)
        return hw

    def distribute_torque_rw(self, tau_c: np.ndarray) -> np.ndarray:
        """ Distribute the torque over the reaction wheels. To the best
            of the systems ability.
        """
        # NOTE: Reaction wheel torque for pointing control. Torque
        # is distributed by the pseudo inverse matrix A_inv
        L = np.zeros_like(tau_c)
        u_wheels = self.A_inv @ tau_c   # Torque for each wheel
        for i, ac in enumerate(self.reaction_wheels):
            L += ac.axis * ac.apply_torque(u_wheels[i])
        return L

    def distribute_torque_mqt(self, tau_m: np.ndarray, b: np.ndarray) -> np.ndarray:
        # NOTE: Magnetorquer distribution. The torque provided from magnetorquers
        # does not happen in the same axis as its orientation mqt.axis. It depends
        # on the magnetic field vector, so needs to be accounted for.

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
        """ Calculate the reference dipole for the magnetorquers in order to 
            desaturate the reaction wheels. The method used is the static input allocation
            presented in the following paper:

            Reaction Wheels Desaturation Using Magnetorquers and Static Input Allocation

            Jean-François Tregouet, Denis Arzelier, Dimitri Peaucelle, Christelle Pittet,
            Luca Zaccarian

            Link to paper: https://hal.science/hal-01760720/document
            See equation (25): 
                $$
                    tau_m = -\frac{(R(q)\tilde{b}_{\circ}(t))^{\times}}
                    {|\tilde{b}_{\circ}(t)|^{2}}k_p(h_\omega - h_{ref})
                $$
            or
                $$
                    tau_m = -\frac{\tilde{b}_{\circ}^{\times}(t)}
                    {|\tilde{b}_{\circ}(t)|^{2}}k_p(h_\omega - R(q)^{T} h_{ref})
                $$
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
        """ Given the actuation from the pointing controller u, distribute 
            actuation over the actuation system to achieve pointing accuracy 
            and desaturation of the reaction wheels. This implementation 
            uses static input allocation to achieve desaturation, which is
            independent of the pointing controller, and achieves both pointing
            accuracy and desaturation without affecting the pointning dynamics.

            Input to reaction wheels:
            $$
                tau_w = -omega X h_w + T_m - u
            $$

            where omega is the the rotation of the satellite frame relative 
            to the inertial frame, h_w is the angular momentum of the reaction
            wheels, and T_m is the torque produced by the magnetorquers.
        """
        # Get the necessary parameters for desaturation
        b0 = kwargs.get("b0", np.array([0.0, 0.0, 1.0]))
        # b = kwargs.get("b0", np.array([0.0, 0.0, 1.0]))
        R = kwargs.get("R", np.eye(3))
        w_ref = kwargs.get("w_ref", 1.0)
        kp = kwargs.get("kp", 1.0)
        # kp = 0.0001
        omega = kwargs.get("w", np.zeros(3))

        # b0 = -R @ b

        # Calculate the magnetorquer actuation
        tau_m = self.desaturate(b0, R, w_ref, kp)
        t_m = self.distribute_torque_mqt(tau_m, b=R @ b0)
        T_m = -np.cross(R @ b0, t_m)

        # Calculate the reaction wheel actuation
        hw = self.angular_momentum_wheels()
        tau_w = -misc.skew(omega) @ hw + T_m - u
        # tau_w = T_m - u
        L_w = self.distribute_torque_rw(tau_w)

        return -L_w + T_m - misc.skew(omega) @ hw

    def reset(self):
        """ Sets the speed of the reaction wheels to 0. Any other resetting mechanicsm
            for other types of actuators needs to be implemented here.
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
