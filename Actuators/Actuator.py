import numpy as np
import yaml

# Base class for an actuator. All actuators will have the following 
# attributes:
    # - Axis in BODY frame of which the actuator is pointing

# Actuators will have the method `apply_torque(self, L, dt)` which 
# will calculate the torque the actuator will be capable of provided in the 
# timeframe dt with reference L.

class Actuator:
    def __init__(self, axis:np.ndarray) -> None:
        self.axis = axis

    def apply_torque(self, L, dt):
        return L
