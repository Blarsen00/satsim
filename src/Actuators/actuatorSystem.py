from matplotlib.pyplot import axis
import numpy as np
from Actuators.actuator import Actuator, ActuatorAnimation
from Actuators.reactionwheel import ReactionWheel
from Actuators.magnetorquer import Magnetorquer
from animation import BaseAnimation
from typing import List, Sequence, Optional, Type, Self


IDEAL_ACTUATORS = [
    Actuator(axis=np.array([1.0, 0.0, 0.0])),
    Actuator(axis=np.array([0.0, 1.0, 0.0])),
    Actuator(axis=np.array([0.0, 0.0, 1.0]))
]

BASE_RW_CONFIG = [
    ReactionWheel(axis=np.array([1.0, 0.0, 0.0])),
    ReactionWheel(axis=np.array([0.0, 1.0, 0.0])),
    ReactionWheel(axis=np.array([0.0, 0.0, 1.0])),
    ReactionWheel(axis=np.array([0.0, 0.0, 1.0]))
]


class ActuatorSystem:
    # actuators: Sequence[Actuator] = BASE_RW_CONFIG
    # animations: List[ActuatorAnimation]

    def __init__(self, actuators: Optional[Sequence[Actuator]]=None) -> None:
        # print([x.axis for x in BASE_RW_CONFIG])
        self.actuators: Sequence[Actuator] = BASE_RW_CONFIG if actuators is None else actuators
        self.animations = [ActuatorAnimation(ac) for ac in self.actuators]

        self.A = np.array([x.axis for x in self.actuators]).T
        self.A_inv = np.linalg.pinv(self.A)


    # TODO: Implement desaturation of reaction wheels

    @classmethod
    def init_base_rw_system(cls) -> Self:
        base_rw = [
            ReactionWheel(axis=np.array([1.0, 0.0, 1.0])),
            ReactionWheel(axis=np.array([2.0, 1.0, 0.0])),
            ReactionWheel(axis=np.array([2.0, 1.0, 1.0])),
        ]
        return cls(actuators=base_rw)

    def apply_torque(self, tau: np.ndarray) -> np.ndarray:
        L = np.zeros_like(tau)
        tau_A = self.A_inv @ tau
        for i, ac in enumerate(self.actuators):
            L += ac.axis * ac.apply_torque(tau_A[i])
        return L


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
        # s += f"Det: {np.linalg.det(self.A)}"
        s += f"{[x.actuator.axis for x in self.animations]}"
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
