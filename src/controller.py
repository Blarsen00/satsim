import numpy as np
import misc
from simulation import PhysicalState
from abc import ABC, abstractmethod


class Controller(ABC):
    """
    Abstract base class for a simulation controller.

    This class defines the interface for controllers that calculate the
    actuator output (typically torque) based on the current state and a
    reference state. It also provides mechanisms for managing **tunable parameters**.

    Class Attributes
    ----------------
    _params : list of str
        A class-level list containing the names of the attributes that are
        considered tunable parameters for the controller and will be subject
        to tuning in a tkinter frame.

    Attributes
    ----------
    _params : list of str
        (Inherited) List of tunable parameter names.
    """
    # Class-level list of tunable parameters
    _params = []
    def __init__(self) -> None:
        super().__init__()

    def set_param(self, key: str, value):
        """
        Sets the value of a tunable parameter, checking for existence and type correctness.

        Parameters
        ----------
        key : str
            The name of the parameter to set. Must be in :attr:`_params`.
        value : object
            The new value for the parameter. Its type must match the current
            type of the parameter attribute.

        Raises
        ------
        AssertionError
            If `key` is not in :attr:`_params`.
        AssertionError
            If the type of `value` does not match the existing parameter's type.
        """
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
        """
        Abstract method for calculating the output torque vector :math:`\\mathbf{L}`.

        This method takes the current (estimated) state, and a reference state
        and uses them to calculate the necessary control output, typically
        a torque vector :math:`\\mathbf{L}`.

        Parameters
        ----------
        state : :class:`simulation.PhysicalState`
            The current (estimated) physical state of the body.
        ref : :class:`simulation.PhysicalState`
            The desired reference physical state.
        **kwargs : dict
            Additional arguments required by specific controllers (e.g., Inertia matrix J).

        Returns
        -------
        :class:`numpy.ndarray`
            The calculated output torque vector :math:`\\mathbf{L}`, shape (3,).
        """
        pass

    def __str__(self) -> str:
        """
        Returns a string representation showing the current values of all tunable parameters.
        """
        s = ''
        for key in self._params:
            s += "{:>12}: {}\n".format(key, getattr(self, key))
        return s


class PDController(Controller):
    """
    Implements a simple Proportional-Derivative (PD) controller for attitude control.

    The control law calculates the torque :math:`\\mathbf{L}` based on the quaternion
    error vector and the angular velocity error.

    Parameters
    ----------
    p : float, optional
        Proportional gain :math:`P`. Defaults to 0.1.
    d : :class:`numpy.ndarray`, optional
        Derivative gain vector :math:`D`, shape (3,). Defaults to :math:`[1.0, 1.0, 1.0]`.

    Attributes
    ----------
    p : float
        Proportional gain :math:`P`.
    d : :class:`numpy.ndarray`
        Derivative gain vector :math:`D`.
    """
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
        """
        Calculates the output torque vector :math:`\\mathbf{L}` using the PD control law.

        The control law is approximately:
        :math:`\\mathbf{L} = \\mathrm{sgn}(q_4) P \\mathbf{q}_{vec,e} - D \\mathbf{\\omega}_e`
        where :math:`\\mathbf{q}_{vec,e}` is the error quaternion vector, :math:`q_4` is the
        error quaternion scalar, and :math:`\\mathbf{\\omega}_e` is the angular velocity error.

        Parameters
        ----------
        state : :class:`simulation.PhysicalState`
            The current physical state.
        ref : :class:`simulation.PhysicalState`
            The reference physical state.
        **kwargs : dict
            Ignored for this controller.

        Returns
        -------
        :class:`numpy.ndarray`
            The calculated output torque vector :math:`\\mathbf{L}`, shape (3,).
        """

        qc = ref.rot.as_quat()
        wc = ref.w
        q = state.rot.as_quat()
        w = state.w

        # Error quaternion $\mathbf{q}_e = \mathbf{q}^* \otimes \mathbf{q}_c$
        # Assuming misc.quat_conjugate negates x,y,z components
        qe = misc.quat_multiply(misc.quat_conjugate(q), qc)
        qvec = qe[:-1] # Vector part of error quaternion
        we = w - wc    # Angular velocity error

        # Control torque
        L = np.sign(qe[-1]) * self.p * qvec - self.d * we
        return L


class SMCController(Controller):
    """
    Implements a Sliding Mode Controller (SMC) for attitude control.

    This controller is based on a sliding surface :math:`\\mathbf{s} = \\mathbf{\\omega}_e + k \\mathbf{q}_{vec,e}`
    and uses a saturation function to approximate the sign function.

    Parameters
    ----------
    e : float, optional
        The boundary layer thickness :math:`\\epsilon`. Defaults to 0.01.
    k : float, optional
        The positive gain :math:`k` used in the sliding surface definition. Defaults to 0.015.
    G : :class:`numpy.ndarray`, optional
        The diagonal gain matrix :math:`\\mathbf{G}`, shape (3, 3). Defaults to a diagonal
        matrix with 0.15 on the main diagonal.

    Attributes
    ----------
    e : float
        The boundary layer thickness :math:`\\epsilon`.
    k : float
        The positive gain :math:`k` for the sliding surface.
    G : :class:`numpy.ndarray`
        The diagonal gain matrix :math:`\\mathbf{G}`.
    """
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
    def saturation(manifold: np.ndarray, epsilon: float) -> np.ndarray:
        """
        Implements the saturation function in (7.25), where the manifold
        is assumed to be the manifold generated by (7.23a). Epsilon is 
        also assumed to be constant over the whole manifold, which 
        may or may not hold.

        The function returns:
        :math:`1`, if :math:`s_i > \\epsilon`
        :math:`s_i / \\epsilon`, if :math:`|s_i| \\le \\epsilon`
        :math:`-1`, if :math:`s_i < -\\epsilon`

        Parameters
        ----------
        manifold : :class:`numpy.ndarray`
            The sliding surface vector :math:`\\mathbf{s}`, shape (N,).
        epsilon : float
            The boundary layer thickness :math:`\\epsilon`.

        Returns
        -------
        :class:`numpy.ndarray`
            The saturated vector :math:`\\mathbf{s}`, shape (N,).
        """
        s = manifold

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
        """
        Calculates the output torque vector :math:`\\mathbf{L}` using the Sliding Mode Control law.

        The control torque is calculated based on the SMC equation that includes the
        inertia matrix, desired acceleration, error terms, and the saturation
        function of the sliding manifold.

        Parameters
        ----------
        state : :class:`simulation.PhysicalState`
            The current physical state.
        ref : :class:`simulation.PhysicalState`
            The reference physical state.
        **kwargs : dict
            Must contain the inertia matrix :math:`\\mathbf{J}` under the key "J".

        Returns
        -------
        :class:`numpy.ndarray`
            The calculated output torque vector :math:`\\mathbf{L}`, shape (3,).

        Raises
        ------
        KeyError
            If the inertia matrix :math:`\\mathbf{J}` is not provided in `kwargs`.
        """

        J: np.ndarray = kwargs["J"]
        q = state.rot.as_quat()
        e = self.e
        k = self.k
        G = self.G

        # Error quaternion $\mathbf{q}_e = \mathbf{q}_c \otimes \mathbf{q}^*$
        qe = misc.quat_multiply(ref.rot.as_quat(), misc.quat_conjugate(q))
        q_vec = qe[:-1]
        q4 = qe[-1]

        # Sliding manifold $\mathbf{s} = \mathbf{\omega}_e + k \mathbf{q}_{vec,e}$
        s_manifold = (state.w - ref.w) + k * q_vec
        s_bar = SMCController.saturation(s_manifold, e)

        # Moment generated from sliding mode controller (based on equation 7.23b)
        # NOTE: The original code uses .T which might imply transpose but is often redundant for 1D numpy arrays.
        L = J @ (k / 2 * (abs(q4) * (ref.w - state.w) - \
                          misc.skew(q_vec) @ (ref.w - state.w).T) \
                          + ref.w_dot - G @ s_bar.T) \
                          + misc.skew(state.w) @ J @ state.w.T
        return L.ravel()
