import numpy as np
from scipy.spatial.transform import Rotation
import misc

from dataclasses import dataclass, field


@dataclass
class PhysicalState:
    rot: Rotation = field(default_factory=lambda: Rotation.from_matrix(np.identity(3)))    # Rotation
    w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    w_dot: np.ndarray = field(default_factory=lambda: np.zeros(3))


class Simulate:
    @staticmethod
    def calculate_angular_momentum(L: np.ndarray, w: np.ndarray,
                                H: np.ndarray, dt:float):
        assert type(L) is np.ndarray and L.shape == (3,)
        assert type(w) is np.ndarray and w.shape == (3,)
        assert type(H) is np.ndarray and H.shape == (3,3)
        assert dt > 0.0

        H_dot = L - misc.skew(w) * H
        return H + dt * H_dot

    @staticmethod
    def direct_euler(w: np.ndarray, w_dot: np.ndarray, dt: float):
        """
            Performs a direct Euler integration step. Primerily intended
            for angular velocity in this context.
        """
        assert w.shape == w_dot.shape
        assert dt > 0.0

        w += w_dot * dt
        return w

    @staticmethod
    def calculate_attitude(rot: Rotation, w: np.ndarray, dt: float):
        assert type(rot) is Rotation
        assert type(w) is np.ndarray and w.shape == (3,)
        assert dt > 0.0

        qw = np.array([*w, 0])
        q_dot = 1/2 * misc.quat_multiply(rot.as_quat(), qw)
        q = rot.as_quat() + dt * q_dot
        return Rotation.from_quat(q)

    @staticmethod
    def update_state(L: np.ndarray, state: PhysicalState, 
                     J: np.ndarray, dt: float) -> PhysicalState:
        state.w_dot = Simulate.calculate_angular_accelleration(L, state.w, J)
        state.w = Simulate.direct_euler(state.w, state.w_dot, dt)
        state.rot = Simulate.calculate_attitude(state.rot, state.w, dt)
        return state


    @staticmethod
    def calculate_angular_accelleration(L: np.ndarray, w: np.ndarray, J: np.ndarray):
        """
            Eulers rotational equation used to calculate the angular accelleration.
        """
        J_inv = np.linalg.inv(J)
        w_dot = J_inv @ (L - misc.skew(w) @ J @ w)
        return w_dot


