import numpy as np
from scipy.spatial.transform import Rotation
import misc

from dataclasses import dataclass, field


@dataclass
class PhysicalState:
    rot: Rotation = field(default_factory=lambda: Rotation.from_matrix(np.identity(3)))    # Rotation
    w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    w_dot: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def randomize_attitude(self):
        self.rot = Rotation.random()

    def randomize_angular_velocity(self):
        self.w = np.random.uniform(-1, 1, 3)


class Simulate:
    @staticmethod
    def rad2rpm(w: float):
        return w * 60 / 2 * np.pi

    @staticmethod
    def rpm2rad(rpm: float):
        return rpm * 2 * np.pi / 60

    @ staticmethod
    def angular_momentum(J: np.ndarray, w: np.ndarray):
        assert type(J) is np.ndarray and J.shape == (3,3)
        assert type(w) is np.ndarray and w.shape == (3,)
        return J @ w

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
        assert len(w) == len(w_dot)
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
    def quaternion_integration_taylor_expansion(Q: np.ndarray, w: np.ndarray, dt: float):
        assert type(Q) is np.ndarray and Q.shape == (4,)
        assert type(w) is np.ndarray and w.shape == (3,)

        # theta = [2*x*dt for x in w]
        theta = [x*dt for x in w]
        D = np.sum([x**2 for x in theta])
        R = np.array([
            theta[0] * Q[3] - theta[1] * Q[2] + theta[2] * Q[1],
            theta[0] * Q[2] + theta[1] * Q[3] - theta[2] * Q[0],
            theta[0] * Q[1] + theta[1] * Q[0] + theta[2] * Q[3],
            theta[0] * Q[0] - theta[1] * Q[1] - theta[2] * Q[3]
        ])

        Q = Q + 0.5 * R - (D - D/3 - D/4) * Q

        return Rotation.from_quat(Q)

    @staticmethod
    def update_state(L: np.ndarray, state: PhysicalState, 
                     J: np.ndarray, dt: float) -> PhysicalState:
        state.w_dot = Simulate.calculate_angular_accelleration(L, state.w, J)
        state.w = Simulate.direct_euler(state.w, state.w_dot, dt)
        state.rot = Simulate.calculate_attitude(state.rot, state.w, dt)
        # state.rot = Simulate.quaternion_integration_taylor_expansion(state.rot.as_quat(), state.w, dt)
        return state


    @staticmethod
    def calculate_angular_accelleration(L: np.ndarray, w: np.ndarray, J: np.ndarray):
        """
            Eulers rotational equation used to calculate the angular accelleration.
        """
        J_inv = np.linalg.inv(J)
        w_dot = J_inv @ (L - misc.skew(w) @ J @ w)
        return w_dot


    # def update(self, frame):
    #     # Calculate
    #     L = self.controller.output(self.sat.state, self.ref.state, J=self.sat.J)
    #     self.sat.state = Simulate.update_state(L,
    #                                            self.sat.state,
    #                                            self.sat.J,
    #                                            self.parameters_animation.dt)
    #     self.ref.update(self.parameters_animation.dt)
    #
    #     # Draw
    #     self.draw_satellite()
    #     if self.parameters_plot.reference:
    #         self.draw_ref()
    #     return self.axes + self.axes_ref

    # @staticmethod
    # def 


