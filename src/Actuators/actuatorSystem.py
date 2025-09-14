import numpy as np
from actuator import Actuator
from reactionwheel import ReactionWheel
from magnetorquer import Magnetorquer
from typing import List


class ActuatorSystem:
    def __init__(self, actuators: List[Actuator]) -> None:
        self.rw = [x.axis for x in actuators if type(x) is ReactionWheel]
        self.mqt = [x.axis for x in actuators if type(x) is Magnetorquer]

    def control_output(self, L, dt):
        pass

