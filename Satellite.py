import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass, field
from copy import deepcopy

# from Actuators.Reactionwheel import ReactionWheel
# from Actuators.Reactionwheel import Magnetorquer
import misc


@dataclass
class Satellite_State:
    rot: Rotation = field(default_factory=lambda: Rotation.from_matrix(np.identity(3)))    # Rotation
    w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    w_dot: np.ndarray = field(default_factory=lambda: np.zeros(3))


class Satellite:
    def __init__(self) -> None:
        self.beginning_state = Satellite_State()
        self.state = Satellite_State()

        self.J: np.ndarray = np.identity(3) * 0.1        # Inertia matrix
        self.J_inv = np.linalg.inv(self.J)
        self.H: np.ndarray = np.zeros_like(self.J)


    def reset(self):
        self.state = deepcopy(self.beginning_state)


    def load_satellite_parameters(self, state):
        self.beginning_state = deepcopy(state)
        self.state = state


    def calculate_angular_velocity(self, L: np.ndarray, dt: float) -> np.ndarray:
        # (3.81) Update the angular velocity
        self.state.w_dot = self.J_inv @ (L - misc.skew(self.state.w) @ \
                                self.J @ self.state.w)
        return self.state.w + dt * self.state.w_dot


    def calculate_angular_momentum(self, L, dt):
        H_dot = L - misc.skew(self.state.w) * self.H
        return self.H + dt * H_dot


    def calculate_attitude(self, dt) -> Rotation:
        qw = np.array([*self.state.w, 0])
        # q_dot = 1/2 * misc.quat_multiply(qw, self.state.rot.as_quat())
        q_dot = 1/2 * misc.quat_multiply(self.state.rot.as_quat(), qw)
        q = self.state.rot.as_quat() + dt * q_dot
        return Rotation.from_quat(q)


    def update_attitude(self, L, dt) -> None:
        self.H = self.calculate_angular_momentum(L, dt)
        self.state.w = self.calculate_angular_velocity(L, dt)
        self.state.rot = self.calculate_attitude(dt)

