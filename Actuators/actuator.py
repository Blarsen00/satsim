import numpy as np
import matplotlib.pyplot as plt
from abc import abstractmethod
from typing import Optional
import yaml

# Base class for an actuator. All actuators will have the following 
# attributes:
    # - Axis in BODY frame of which the actuator is pointing

# Actuators will have the method `apply_torque(self, L, dt)` which 
# will calculate the torque the actuator will be capable of provided in the 
# timeframe dt with reference L.


class Actuator:
    axis: np.ndarray = np.array([1.0, 0.0, 0.0])

    def __init__(self, axis: Optional[np.ndarray]=None) -> None:
        self.axis = np.array([1.0, 0.0, 0.0]) if axis is None else axis
        self.param = {}

    @abstractmethod
    def load_from_yaml(self, filepath: str):
        with open(filepath, 'r') as file:
            self.param = yaml.safe_load(file)
        file.close()

    @abstractmethod
    def apply_torque(self, tau: float, dt: float):
        """
            Calculate and return the torque from the actuator
            provided referece torque.
        """
        return L

    @staticmethod
    def saturate(value: float,
                 maximum: Optional[float]=None,
                 minimum: Optional[float]=None):

        maximum = maximum if maximum is not None else np.infty
        minimum = minimum if minimum is not None else -np.infty

        return min(maximum, max(value, minimum))


class Magnetorquer(Actuator):
    # TODO: Calculate the magnetic field strength based on the position of the satellite
    # TODO: Calculate torque based on the strength of the magnetic field
    # TODO: Calculate energy output from current etc.
    # TODO: Calculate torque from current available

    maxTorque: float = 1.0
    maxCurrent: float = 1.0
    scalingFactor: float = 1.0

    def __init__(self, axis: Optional[np.ndarray]=None) -> None:
        super().__init__(axis)

    def apply_torque(self, L, dt):
        """
            The torque provided from the magnettorquer is essentially instant, 
            and thus does not need any model of the transient response, and 
            just the statuarted output does just fine for now.
        """
        return Actuator.saturate(L, self.maxTorque, -self.maxTorque)




if __name__ == "__main__":
    pass
