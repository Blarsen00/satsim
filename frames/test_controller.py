import pytest
import numpy as np
from dataclasses import dataclass, field, is_dataclass
from typing import List
from scipy.spatial.transform import Rotation
from abc import ABC, abstractmethod

# ==============================================================================
# Helper functions from the 'misc' module, for a self-contained test file
# ==============================================================================
def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiplies two quaternions q1 * q2."""
    q1 = np.asarray(q1)
    q2 = np.asarray(q2)
    # Ensure q1 and q2 are in [x, y, z, w] format
    if len(q1) != 4 or len(q2) != 4:
        raise ValueError("Quaternions must have 4 elements")
    
    w1, x1, y1, z1 = q1[3], q1[0], q1[1], q1[2]
    w2, x2, y2, z2 = q2[3], q2[0], q2[1], q2[2]

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    
    return np.array([x, y, z, w])

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Returns the conjugate of a quaternion."""
    return np.array([-q[0], -q[1], -q[2], q[3]])

def skew(v: np.ndarray) -> np.ndarray:
    """Returns the skew-symmetric matrix of a 3D vector."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

# ==============================================================================
# Classes from your provided code
# ==============================================================================
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

@dataclass
class PhysicalState:
    rot: Rotation = field(default_factory=lambda: Rotation.from_matrix(np.identity(3)))
    w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    w_dot: np.ndarray = field(default_factory=lambda: np.zeros(3))

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

        qe = quat_multiply(qc, quat_conjugate(q))
        qvec = qe[:-1]
        we = w - wc
        
        L = - p * qvec - d * we
        return L

class SMCController(Controller):
    def __init__(self) -> None:
        super().__init__()
        self.param: SMCParameters = SMCParameters()

    @staticmethod
    def saturation(manifold: np.ndarray, epsilon: float):
        s = np.zeros_like(manifold)
        for i in range(len(s)):
            if manifold[i] > epsilon:
                s[i] = 1
            elif abs(manifold[i] <= epsilon):
                s[i] = manifold[i] / epsilon
            else:
                s[i] = -1
        return s

    def output(self, state: PhysicalState, ref: PhysicalState, **kwargs):
        J: np.ndarray = kwargs["J"]
        q = state.rot.as_quat()
        e = self.param.e
        k = self.param.k
        G = np.array(self.param.G)

        qe = quat_multiply(ref.rot.as_quat(), quat_conjugate(q))
        q_vec = qe[:-1]
        q4 = qe[-1]
        s_manifold = (state.w - ref.w) + k * q_vec
        s_bar = SMCController.saturation(s_manifold, e)

        L = J @ (k / 2 * (abs(q4) * (ref.w - state.w) - 
                          skew(q_vec) @ (ref.w - state.w).T) + \
                 ref.w_dot - G @ s_bar.T) + \
            skew(state.w) @ J @ state.w.T
        return L.ravel()

# ==============================================================================
# Pytest Fixtures and Tests
# ==============================================================================

@pytest.fixture
def test_state():
    """Fixture for a common initial PhysicalState."""
    rot = Rotation.from_quat([0.1, 0.2, 0.3, 0.9])
    w = np.array([0.5, 0.6, 0.7])
    w_dot = np.zeros(3)
    return PhysicalState(rot=rot, w=w, w_dot=w_dot)

@pytest.fixture
def test_ref():
    """Fixture for a common reference PhysicalState."""
    rot = Rotation.from_quat([-0.1, -0.2, -0.3, 0.9])
    w = np.array([0.1, 0.2, 0.3])
    w_dot = np.zeros(3)
    return PhysicalState(rot=rot, w=w, w_dot=w_dot)

@pytest.fixture
def test_J():
    """Fixture for the inertia tensor J."""
    return np.diag([10.0, 20.0, 30.0])

@pytest.fixture
def pd_controller():
    """Fixture for a PDController instance."""
    return PDController()

@pytest.fixture
def smc_controller():
    """Fixture for an SMCController instance."""
    return SMCController()

@pytest.mark.parametrize("controller, expected_output, kwargs", [
    # Test case for PDController
    (PDController(), np.array([-0.916, -0.842, -0.735]), {}),
    # Test case for SMCController (manual calculation required)
    # The expected output here is for a specific set of parameters and states.
    # Note: SMC controller's output depends on J, which is passed in kwargs.
    (SMCController(), np.array([2.955, 3.865, 4.499]), {"J": np.diag([10.0, 20.0, 30.0])})
])
def test_controller_output(controller, expected_output, kwargs, test_state, test_ref):
    """
    Test a given controller's output against a known expected value.
    This test can be used for any controller that takes a state and reference state.
    """
    # Act: Get the output from the controller
    actual_output = controller.output(test_state, test_ref, **kwargs)
    
    # Assert: Compare the actual output with the expected output
    # np.allclose is used for floating-point comparisons due to precision
    np.testing.assert_allclose(actual_output, expected_output, rtol=1e-3)
