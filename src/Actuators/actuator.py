import numpy as np
import matplotlib.pyplot as plt
from matplotlib.artist import Artist
from abc import abstractmethod
from typing import Optional, Union, Iterable
import yaml
from collections import defaultdict, deque
from animation import BaseAnimation

# Base class for an actuator. All actuators will have the following 
# attributes:
    # - Axis in BODY frame of which the actuator is pointing

# Actuators will have the method `apply_torque(self, L, dt)` which 
# will calculate the torque the actuator will be capable of provided in the 
# timeframe dt with reference L.


class Actuator():
    # axis: np.ndarray = np.array([1.0, 0.0, 0.0])
    history_size: int = 100

    def __init__(self, axis: Optional[np.ndarray]=None) -> None:
        # Normalize the axis
        self.axis = np.array([1.0, 0.0, 0.0]) if axis is None else axis / np.linalg.norm(axis)
        self.param = {}
        self.data = defaultdict(lambda: deque(maxlen=self.history_size))
        self.log_data("time", 0.0)

    def log_data(self, key: str, value: Optional[Union[float, np.ndarray]]):
        self.data[key].append(value)

    # @abstractmethod
    def load_from_yaml(self, filepath: str):
        with open(filepath, 'r') as file:
            self.param = yaml.safe_load(file)
        file.close()

    @abstractmethod
    def apply_torque(self, tau: float, dt: float=0.1):
        """
            Calculate and return the torque from the actuator
            provided referece torque.
        """
        return tau

    @staticmethod
    def saturate(value: float,
                 maximum: Optional[float]=None,
                 minimum: Optional[float]=None):

        maximum = maximum if maximum is not None else np.inf
        minimum = minimum if minimum is not None else -np.inf

        return min(maximum, max(value, minimum))


##################### Animation part of the actuators ####################
# OBS: Plotting parameters for each key
PLOTTING_PARAMETES = {
    "torque": {
        # "title": "Reaction Wheel",
        "linestyle": "-",
        "x_label": "Time (s)",
        "y_label": r"$\tau$ (Nm)",
        "y_lim": (-1.0e-3, 1.0e-3),
        "grid": True,
        "legend": False
    },
    "reference": {
        # "title": "Reaction Wheel",
        "linestyle": "--",
        "x_label": "Time (s)",
        "y_label": r"$\tau_{ref}$ (Nm)",
        "y_lim": (-1.0e-3, 1.0e-3),
        "grid": True,
        "legend": False
    },
    "drag": {
        # "title": "Reaction Wheel",
        "linestyle": "-",
        "x_label": "Time (s)",
        "y_label": r"$\tau_{d}$ (Nm)",
        "y_lim": (-1.0e-3, 1.0e-3),
        "grid": True,
        "legend": False
    },
    "motor": {
        # "title": "Reaction Wheel",
        "linestyle": "-",
        "x_label": "Time (s)",
        "y_label": r"$\tau_{m}$ (Nm)",
        "y_lim": (-1.0e-3, 1.0e-3),
        "grid": True,
        "legend": False
    },
    "w": {
        # "title": "Reaction Wheel",
        "linestyle": "-",
        "x_label": "Time (s)",
        "y_label": r"$\omega$ ($\frac{rad}{s}$)",
        "y_lim": (0, 1.0),
        "grid": True,
        "legend": False
    },
    "w_dot": {
        # "title": "Reaction Wheel",
        "linestyle": "-",
        "x_label": "Time (s)",
        "y_label": r"$\omega$ ($\frac{rad}{s}$)",
        "y_lim": (0, 1.0),
        "grid": True,
        "legend": False
    },
    "rpm": {
        # "title": "Reaction Wheel",
        "linestyle": "-",
        "x_label": "Time (s)",
        "y_label": r"$\omega$ (rpm)",
        "y_lim": (0, 5000),
        "grid": True,
        "legend": False
    },
    "I": {
        # "title": "Reaction Wheel",
        "linestyle": "-",
        "x_label": "Time (s)",
        "y_label": "Current (A)",
        "y_lim": (0, 1),
        "grid": True,
        "legend": False
    },
}


class ActuatorAnimation(BaseAnimation):
    def __init__(self, actuator) -> None:
        self.actuator = actuator

        # WARN: This needs to happen before calling super()__init__() for BaseAnimation
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot()
        self.color = "blue"
        self.key = "torque"
        # self.key = "w"
        # self.key = "rpm"

        super().__init__()

    def add_canvas(self, canvas):
        self.canvas = canvas

    def draw(self):
        return super().draw()

    def update_display_values(self):
        for key, var in self.display_vars.items():
            # param = PLOTTING_PARAMETES[key]
            value = self.actuator.data[key][-1]
            txt = f"{key}: {value:.2f}"
            var.set(txt)

    def init_anim(self) -> Iterable[Artist]:
        param = PLOTTING_PARAMETES[self.key]
        # self.ax.set_xlabel(param["x_label"])
        self.ax.set_ylabel(param["y_label"])
        self.ax.set_ylim(param["y_lim"])
        self.ax.grid(param["grid"])
        if param["legend"]:
            self.ax.legend()

        self.line, = self.ax.plot(
            [], [],
            self.color,
            linestyle=param["linestyle"],
            label=param["y_label"],
            linewidth=2
        )
        # Only torque has a reference at this time
        if self.key == "torque":
            self.ref_line, = self.ax.plot(
                [], [], 
                color=self.color,
                linewidth=2,
                linestyle=PLOTTING_PARAMETES["reference"]["linestyle"],
                label=PLOTTING_PARAMETES["reference"]["y_label"]
            )
            return (self.line, self.ref_line, )
        return (self.line, )

    def update_anim(self, frame: int) -> Iterable[Artist]:
        x = self.actuator.data["time"]
        y = self.actuator.data[self.key]

        self.line.set_data(x, y)
        self.ax.set_xlim(x[0], x[-1] + 3.0)

        self.update_display_values()

        if self.key == "torque":
            y_ref = self.actuator.data["reference"]
            self.ref_line.set_data(x, y_ref)
            return (self.line, self.ref_line, )

        return (self.line, )


if __name__ == "__main__":
    pass

