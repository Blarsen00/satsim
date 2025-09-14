import numpy as np
import misc
from simulation import PhysicalState
from abc import ABC, abstractmethod
from dataclasses import dataclass, is_dataclass, field
from typing import List


@dataclass
class PDParameters:
    p: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    d: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])


@dataclass
class SMCParameters:
    G: List[List[float]] = field(default_factory=lambda:[[0.15, 0.0, 0.0],
                                                         [0.0, 0.15, 0.0],
                                                         [0.0, 0.0, 0.15]])
    k: float = 0.015
    e: float = 0.01


class Controller(ABC):
    def __init__(self) -> None:
        pass

    @staticmethod
    def get_controller(controller_parameters):
        if isinstance(controller_parameters, SMCParameters):
            controller = SMCController()
            controller.load_params(controller_parameters)
            return controller

        elif isinstance(controller_parameters, PDParameters):
            controller = PDController()
            controller.load_params(controller_parameters)
            return controller

        raise ValueError(f"Unknown controller type: {controller_parameters}")


    def load_params(self, param):
        if is_dataclass(param):
            self.param = param

    def load_state(self, state: PhysicalState) -> None:
        self.state = state

    def load_ref(self, ref: PhysicalState) -> None:
        self.ref = ref


    @abstractmethod
    def output(self, state: PhysicalState, ref: PhysicalState, **kwargs) -> np.ndarray:
        """ Base method to be overriden in subclass """
        pass


class PDController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self.param: PDParameters = PDParameters()


    def output(self, state: PhysicalState, ref: PhysicalState, **kwargs):
        qc = ref.rot.as_quat()
        wc = ref.w
        q = state.rot.as_quat()
        w = state.w
        p = np.array(self.param.p)
        d = np.array(self.param.d)

        # qe = misc.quat_multiply(qc, misc.quat_conjugate(q))
        qe = misc.quat_multiply(misc.quat_conjugate(q), qc)
        qvec = qe[:-1]
        we = w - wc
        # we = wc - w

        # L = - np.sign(qe[-1]) *  p * qvec - d * we
        L = np.sign(qe[-1]) *  p * qvec - d * we
        # L = - p * qvec - d * we
        # L = - p * qvec - d * w
        return L


class SMCController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self.param: SMCParameters = SMCParameters()


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


    def output(self, state: PhysicalState, ref: PhysicalState, **kwargs):
        J: np.ndarray = kwargs["J"]
        q = state.rot.as_quat()
        e = self.param.e
        k = self.param.k
        G = self.param.G

        qe = misc.quat_multiply(ref.rot.as_quat(), misc.quat_conjugate(q))
        # qe = misc.quat_multiply(misc.quat_conjugate(q), ref.rot.as_quat())
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


