import numpy as np
from scipy.spatial.transform import Rotation
import misc

from dataclasses import dataclass, field


@dataclass
class PhysicalState:
    """
    A class to represent the physical state (attitude and angular velocity)
    of a body.

    Attributes
    ----------
    rot : :class:`scipy.spatial.transform.Rotation`
        The current rotation (attitude) of the body. Default is identity rotation.
    w : :class:`numpy.ndarray`
        The angular velocity vector $\mathbf{w}$ in rad/s, shape (3,).
        Default is $\mathbf{0}$.
    w_dot : :class:`numpy.ndarray`
        The angular acceleration vector $\mathbf{\dot{w}}$ in rad/s^2, shape (3,).
        Default is $\mathbf{0}$.
    """
    rot: Rotation = field(default_factory=lambda: Rotation.from_matrix(np.identity(3)))
    w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    w_dot: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def randomize_attitude(self):
        """
        Sets the rotation attribute to a random, uniformly distributed attitude.
        """
        self.rot = Rotation.random()

def randomize_angular_velocity(self, w: float = 1.0):
        """
        Randomizes the angular velocity vector $\mathbf{w}$ with uniform
        distribution between :math:`-w` and :math:`w` rad/s for each component.

        Parameters
        ----------
        w : float, optional
            The maximum absolute value for the angular velocity components
            in rad/s. The range will be :math:`[-w, w]`.
            Defaults to 1.0.
        """
        self.w = np.random.uniform(-w, w, 3)

    def __str__(self) -> str:
        """
        Returns a string representation of the PhysicalState.
        """
        s = f"Rotation: \n{self.rot}\n"
        s += f"w: {self.w}"
        return s


class Simulate:
    """
    A collection of static methods for physical simulations, primarily
    focused on rotational dynamics and conversions.
    """
    @staticmethod
    def rad2rpm(w: float):
        """
        Converts angular velocity from radians per second (rad/s) to
        revolutions per minute (RPM).

        Parameters
        ----------
        w : float
            Angular velocity in rad/s.

        Returns
        -------
        float
            Angular velocity in RPM.
        """
        return w * 60 / (2 * np.pi)

    @staticmethod
    def rpm2rad(rpm: float):
        """
        Converts angular velocity from revolutions per minute (RPM) to
        radians per second (rad/s).

        Parameters
        ----------
        rpm : float
            Angular velocity in RPM.

        Returns
        -------
        float
            Angular velocity in rad/s.
        """
        return rpm * (2 * np.pi) / 60

    @staticmethod
    def angular_momentum(J: np.ndarray, w: np.ndarray):
        """
        Calculates the angular momentum $\mathbf{H}$ using the inertia tensor $\mathbf{J}$
        and angular velocity $\mathbf{w}$: $\mathbf{H} = \mathbf{J}\mathbf{w}$.

        Parameters
        ----------
        J : :class:`numpy.ndarray`
            The inertia tensor, shape (3, 3).
        w : :class:`numpy.ndarray`
            The angular velocity vector $\mathbf{w}$, shape (3,).

        Returns
        -------
        :class:`numpy.ndarray`
            The angular momentum vector $\mathbf{H}$, shape (3,).
        """
        assert type(J) is np.ndarray and J.shape == (3, 3)
        assert type(w) is np.ndarray and w.shape == (3,)
        return J @ w

    @staticmethod
    def calculate_angular_momentum(L: np.ndarray, w: np.ndarray,
                                   H: np.ndarray, dt: float):
        """
        Performs a step-wise update of the angular momentum vector $\mathbf{H}$
        using the relationship $\mathbf{\dot{H}} = \mathbf{L} - \mathbf{\omega} \times \mathbf{H}$.

        Parameters
        ----------
        L : :class:`numpy.ndarray`
            The external torque vector $\mathbf{L}$, shape (3,).
        w : :class:`numpy.ndarray`
            The angular velocity vector $\mathbf{w}$, shape (3,).
        H : :class:`numpy.ndarray`
            The current angular momentum vector $\mathbf{H}$, shape (3,).
        dt : float
            The time step for integration. Must be positive.

        Returns
        -------
        :class:`numpy.ndarray`
            The updated angular momentum vector $\mathbf{H}$ after time $dt$.
        """
        assert type(L) is np.ndarray and L.shape == (3,)
        assert type(w) is np.ndarray and w.shape == (3,)
        assert type(H) is np.ndarray and H.shape == (3,) 
        assert dt > 0.0

        H_dot = L - misc.skew(w) @ H
        return Simulate.direct_euler(H, H_dot, dt)

    @staticmethod
    def direct_euler(x: np.ndarray, x_dot: np.ndarray, dt: float):
        """
        Performs a direct Euler integration step for a state vector,
        typically used for angular velocity in this context: $\mathbf{x}_{k+1} = \mathbf{x}_k + \mathbf{\dot{x}}_k dt$.

        Parameters
        ----------
        x : :class:`numpy.ndarray`
            The current state vector (e.g., angular velocity $\mathbf{w}$).
        x_dot : :class:`numpy.ndarray`
            The current derivative vector (e.g., angular acceleration $\mathbf{\dot{w}}$).
        dt : float
            The time step for integration. Must be positive.

        Returns
        -------
        :class:`numpy.ndarray`
            The updated state vector (e.g., angular velocity $\mathbf{w}$) after time $dt$.
        """
        assert len(x) == len(x_dot)
        assert dt > 0.0

        x += x_dot * dt
        return x

    @staticmethod
    def quaternion_derivate(q: np.ndarray, w: np.ndarray):
        """
        Calculation of the time derivative of a quaternion $\mathbf{\dot{q}}$
        based on the rotation and angular velocity.

        Parameters
        ----------
        q : :class:`numpy.ndarray`
            The current attitude as a quaternion.
        w : :class:`numpy.ndarray`
            The angular velocity vector $\mathbf{w}$, shape (3,).
        """
        qw = np.array([*w, 0])
        q_dot = 1/2 * misc.quat_multiply(q, qw)
        return q_dot

    @staticmethod
    def calculate_attitude(rot: Rotation, w: np.ndarray, dt: float):
        """
        Calculates the updated attitude (rotation) using a simple Euler
        integration of the quaternion derivative $\mathbf{\dot{q}} = \frac{1}{2} \mathbf{q} \otimes [0, \mathbf{w}]^T$.

        Parameters
        ----------
        rot : :class:`scipy.spatial.transform.Rotation`
            The current rotation object.
        w : :class:`numpy.ndarray`
            The angular velocity vector $\mathbf{w}$, shape (3,).
        dt : float
            The time step for integration. Must be positive.

        Returns
        -------
        :class:`scipy.spatial.transform.Rotation`
            The updated rotation object after time $dt$.
        """
        assert type(rot) is Rotation
        assert type(w) is np.ndarray and w.shape == (3,)
        assert dt > 0.0

        # NOTE: Legacy version:
        # qw = np.array([*w, 0])
        # q_dot = 1/2 * misc.quat_multiply(rot.as_quat(), qw)
        # q = rot.as_quat() + dt * q_dot
        # return Rotation.from_quat(q)

        q = rot.as_quat()
        q_dot = Simulate.quaternion_derivate(q, w)
        q_k = Simulate.direct_euler(q, q_dot, dt)
        return Rotation.from_quat(q_k)

    @staticmethod
    def quaternion_integration_taylor_expansion(Q: np.ndarray, w: np.ndarray, dt: float):
        """
        Calculates the updated attitude using a specific Taylor expansion based
        quaternion integration method.

        Parameters
        ----------
        Q : :class:`numpy.ndarray`
            The current quaternion $\mathbf{q}$ as a numpy array, shape (4,).
        w : :class:`numpy.ndarray`
            The angular velocity vector $\mathbf{w}$, shape (3,).
        dt : float
            The time step for integration.

        Returns
        -------
        :class:`scipy.spatial.transform.Rotation`
            The updated rotation object.
        """
        assert type(Q) is np.ndarray and Q.shape == (4,)
        assert type(w) is np.ndarray and w.shape == (3,)

        theta = [x*dt for x in w]
        D = np.sum([x**2 for x in theta])
        R = np.array([
            theta[0] * Q[3] - theta[1] * Q[2] + theta[2] * Q[1],
            theta[0] * Q[2] + theta[1] * Q[3] - theta[2] * Q[0],
            theta[0] * Q[1] + theta[1] * Q[0] + theta[2] * Q[3],
            theta[0] * Q[0] - theta[1] * Q[1] - theta[2] * Q[3]
        ])

        Q = Q + 0.5 * R - (D - D/3 - D/4) * Q # Note: This line looks non-standard for typical Taylor expansion methods
        return Rotation.from_quat(Q)

    @staticmethod
    def update_state(L: np.ndarray, state: PhysicalState,
                     J: np.ndarray, dt: float) -> PhysicalState:
        """
        Performs a full step update of the :class:`PhysicalState` (angular velocity
        and attitude) based on external torque and inertia.

        Parameters
        ----------
        L : :class:`numpy.ndarray`
            The external torque vector $\mathbf{L}$, shape (3,).
        state : :class:`PhysicalState`
            The current physical state of the body.
        J : :class:`numpy.ndarray`
            The inertia tensor $\mathbf{J}$, shape (3, 3).
        dt : float
            The time step for integration.

        Returns
        -------
        :class:`PhysicalState`
            The updated physical state object.
        """
        state.w_dot = Simulate.calculate_angular_accelleration(L, state.w, J)
        state.w = Simulate.direct_euler(state.w, state.w_dot, dt)
        state.rot = Simulate.calculate_attitude(state.rot, state.w, dt)
        # state.rot = Simulate.quaternion_integration_taylor_expansion(state.rot.as_quat(), state.w, dt)
        return state


    @staticmethod
    def calculate_angular_accelleration(L: np.ndarray, w: np.ndarray, J: np.ndarray):
        """
        Calculates the angular acceleration $\mathbf{\dot{w}}$ using Euler's rotational equation:
        $\mathbf{\dot{w}} = \mathbf{J}^{-1} (\mathbf{L} - \mathbf{\omega} \times (\mathbf{J}\mathbf{\omega}))$.

        Parameters
        ----------
        L : :class:`numpy.ndarray`
            The external torque vector $\mathbf{L}$, shape (3,).
        w : :class:`numpy.ndarray`
            The angular velocity vector $\mathbf{w}$, shape (3,).
        J : :class:`numpy.ndarray`
            The inertia tensor $\mathbf{J}$, shape (3, 3).

        Returns
        -------
        :class:`numpy.ndarray`
            The angular acceleration vector $\mathbf{\dot{w}}$, shape (3,).
        """
        J_inv = np.linalg.inv(J)
        w_dot = J_inv @ (L - misc.skew(w) @ J @ w)
        return w_dot
