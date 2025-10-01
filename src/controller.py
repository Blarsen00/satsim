import numpy as np
import misc
from simulation import PhysicalState
from abc import ABC, abstractmethod
from dataclasses import dataclass, is_dataclass, field
from typing import List


class Controller(ABC):
    # Class-level list of tunable parameters
    _params = []
    def __init__(self) -> None:
        super().__init__()

    def set_param(self, key: str, value):
        assert key in self._params, \
            f"{key} is not in the tunable list: {self._params}"
        assert isinstance(value, type(getattr(self, key))), \
            f"The type of value: {type(value)} needs to be of type: {type(getattr(self, key))}"
        setattr(self, key, value)

    @abstractmethod
    def output(self,
               state: PhysicalState,
               ref: PhysicalState,
               **kwargs) -> np.ndarray:
        """ Method for calculating the output for the actuators. Takes
            the current (estimated) state, and reference and uses it to
            calculate the output torque vector.
        """
        pass

    def __str__(self) -> str:
        s = ''
        for key in self._params:
            s += "{:>12}: {}\n".format(key, getattr(self, key))
        return s


class PDController(Controller):
    _params = ['p', 'd']

    def __init__(self, 
                 p: float=0.1,
                 d: np.ndarray=np.array([1.0, 1.0, 1.0])) -> None:
        super().__init__()

        self.p = p
        self.d = d


    def output(self,
               state: PhysicalState,
               ref: PhysicalState,
               **kwargs) -> np.ndarray:

        qc = ref.rot.as_quat()
        wc = ref.w
        q = state.rot.as_quat()
        w = state.w

        qe = misc.quat_multiply(misc.quat_conjugate(q), qc)
        qvec = qe[:-1]
        we = w - wc

        L = np.sign(qe[-1]) * self.p * qvec - self.d * we
        return L


class SMCController(Controller):
    _params = ['e', 'k', 'G']
    def __init__(self) -> None:
        super().__init__()

        self.e = 0.01
        self.k = 0.015
        self.G = np.array([
            [0.15, 0.0, 0.0],
            [0.0, 0.15, 0.0],
            [0.0, 0.0, 0.15]
        ])

    @staticmethod
    def saturation(manifold: np.ndarray, epsilon: float):
        s = np.zeros_like(manifold)

        for i in range(len(s)):
            if s[i] > epsilon:
                s[i] = 1
            elif abs(s[i] <= epsilon):
                s[i] = s[i] / epsilon
            else:
                s[i] = -1
        return s


    def output(self,
               state: PhysicalState,
               ref: PhysicalState,
               **kwargs) -> np.ndarray:

        J: np.ndarray = kwargs["J"]
        q = state.rot.as_quat()
        e = self.e
        k = self.k
        G = self.G

        qe = misc.quat_multiply(ref.rot.as_quat(), misc.quat_conjugate(q))
        q_vec = qe[:-1]
        q4 = qe[-1]

        s_manifold = (state.w - ref.w) + k * q_vec
        s_bar = SMCController.saturation(s_manifold, e)

        # Moment generated from sliding mode controller (7.23b)
        L = J @ (k / 2 * (abs(q4) * (ref.w - state.w) - \
                    misc.skew(q_vec) @ (ref.w - state.w).T) \
                    + ref.w_dot - G @ s_bar.T) \
                    + misc.skew(state.w) @ J @ state.w.T
        return L.ravel()


