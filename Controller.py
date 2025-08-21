import numpy as np
import misc
from parameters import ReferenceParameters, PDParameters, SMCParameters
from Satellite import Satellite_State
from abc import ABC, abstractmethod


class Controller(ABC):
    def __init__(self) -> None:
        self.state: Satellite_State = Satellite_State()
        self.ref: ReferenceParameters = ReferenceParameters()


    @staticmethod
    def get_controller(controller_type: str):
        if controller_type == "smc":
            return SMCController()
        if controller_type == "pd":
            return PDController()
        raise ValueError(f"Unknown controller type: {controller_type}")


    def load_state(self, state: Satellite_State) -> None:
        self.state = state


    def load_ref(self, ref: ReferenceParameters) -> None:
        self.ref = ref


    @abstractmethod
    def output(self, state: Satellite_State, ref: ReferenceParameters, **kwargs) -> np.ndarray:
        """ Base method to be overriden in subclass """
        pass


class PDController(Controller):
    def __init__(self) -> None:
        super().__init__()
        param: PDParameters = PDParameters()
        self.load_params(param)


    def load_params(self, param: PDParameters):
        self.p = np.array([param.P_x, param.P_y, param.P_z])
        self.d = np.array([param.D_x, param.D_y, param.D_z])


    def output(self, state: Satellite_State, ref: ReferenceParameters, **kwargs):
        qc = ref.rot.as_quat()
        wc = ref.W
        q = state.rot.as_quat()
        w = state.w

        qe = misc.quat_multiply(qc, misc.quat_conjugate(q))
        # qe = misc.quat_multiply(misc.quat_conjugate(q), qc)
        qvec = qe[:-1]
        we = w - wc

        # L = -np.sign(qe[-1]) *  self.p * qvec - self.d * we
        L = - self.p * qvec - self.d * we
        return L


class SMCController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self.param: SMCParameters = SMCParameters()
        self.load_params(self.param)


    def load_params(self, param: SMCParameters):
        self.G = param.G
        self.e = param.e
        self.k = param.k
        self.param = param


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


    def output(self, state: Satellite_State, ref: ReferenceParameters, **kwargs):
        J: np.ndarray = kwargs["J"]
        q = state.rot.as_quat()

        qe = misc.quat_multiply(ref.rot.as_quat(), misc.quat_conjugate(q))
        # qe = misc.quat_multiply(misc.quat_conjugate(q), ref.rot.as_quat())
        q_vec = qe[:-1]
        q4 = qe[-1]

        s_manifold = (state.w - ref.W) + self.k * q_vec
        s_bar = SMCController.saturation(s_manifold, self.e)

        # Moment generated from sliding mode controller (7.23b)
        L = J @ (self.k / 2 * (abs(q4) * (ref.W - state.w) - \
                    misc.skew(q_vec) @ (ref.W - state.w).T) \
                    + ref.W_dot - self.G @ s_bar.T) \
                    + misc.skew(state.w) @ J @ state.w.T
        return L.ravel()



