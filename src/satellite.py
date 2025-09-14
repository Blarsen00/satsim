import numpy as np
from copy import deepcopy

from simulation import PhysicalState


class Satellite:
    state: PhysicalState = PhysicalState()
    J: np.ndarray = np.identity(3) * 0.1        # Inertia matrix
    H: np.ndarray = np.zeros((3, 3))            # Angular momentum
    energy: int = 100

    def __init__(self, state=None) -> None:
        if state is not None:
            self.state = state
            self.initial_state = state
        else:
            self.initial_state = PhysicalState()


    def reset(self):
        self.state = deepcopy(self.initial_state)


    def load_satellite_parameters(self, state):
        self.initial_state = deepcopy(state)
        self.state = state


