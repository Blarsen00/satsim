import numpy as np
from abc import abstractmethod
from typing import Optional
import yaml

# Base class for an actuator. All actuators will have the following 
# attributes:
    # - Axis in BODY frame of which the actuator is pointing

# Actuators will have the method `apply_torque(self, L, dt)` which 
# will calculate the torque the actuator will be capable of provided in the 
# timeframe dt with reference L.

# PERF: Hello there
# TODO: Hello there
# WARNING: Hello there
# HACK: Hello there
# FIX: Hello there
# NOTE: Hello there


class Actuator:
    def __init__(self, axis: Optional[np.ndarray]=None) -> None:
        self.axis = np.array([1.0, 0.0, 0.0]) if axis is None else axis
        self.param = {}

    @abstractmethod
    def load_from_yaml(self, filepath: str):
        with open(filepath, 'r') as file:
            self.param = yaml.safe_load(file)
        file.close()

    @abstractmethod
    def apply_torque(self, L, dt):
        return L

    @staticmethod
    def saturate(value: float, maximum: float, minimum: float):
        return min(maximum, max(value, minimum))


class ReactionWheel(Actuator):
    J: np.ndarray = np.eye(3) * 0.01    # Inertia matrix
    w: float = 0.0                      # Angular velocity

    dv: float = 1.0                     # Coloumb damping coefficient
    dc: float = 1.0                     # Viscous damping coefficient

    max_rpm: float = 5000.0
    max_tau: float = 5000.0

    def __init__(self, axis: Optional[np.ndarray] = None) -> None:
        super().__init__(axis)


    @staticmethod
    def drag(w: float, dc: float, dv: float):
        """ Wheel friction modeled as a sum of viscous and Coloumb components.
            (4.55)
        """
        return -dv * w - dc * np.sign(w)

    def voltage_to_torque(self, v):
        # These values will probably change. Ask Christoffer.
        # implement these values as default parameters in class later?

        G_d = 1
        K_t = 1

        return min(max(G_d * K_t * v, -self.max_tau), self.max_tau)

    def apply_voltage(self, v, dt):
        self.apply_torque(self.voltage_to_torque(v), dt)


class Magnetorquer(Actuator):
    def __init__(self, axis=[1.0, 0.0, 0.0], file="Actuators/Yaml/Magnettorquer.yaml") -> None:
        super().__init__(axis)

        with open(file, 'r') as file:
            self.param = yaml.safe_load(file)
        file.close()

        self.scaling = self.param["scalingFactor"]
        self.maxCurrent = self.param["maxCurrent"]
        self.maxTorque = self.param["maxTorque"]


    def apply_torque(self, L, dt):
        # Assume instantanious torque, so dt becomes irrelevant
        tau = min(abs(L), self.maxTorque)
        tau *= np.sign(L)
        return tau
