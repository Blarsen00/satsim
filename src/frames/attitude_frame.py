import tkinter as tk
from tkinter import ttk, filedialog

from dataclasses import dataclass, field
from frames.base_frame import BaseParamFrame

from typing import List


@dataclass 
class PlotParameters:
    width: int = 6          # Inches
    height: int = 4         # Inches
    alpha: float = 0.7      # alpha value for references
    reference: bool = True
    colors: List[str] = field(default_factory=lambda: ["red", "blue", "green"])


@dataclass
class TimeParameters:
    t0: float = 0.0         # Start time for the save
    t_end: float = 20.0     # End time for the save
    dt: float = 0.05        # dt for the simulation
    interval: int = 5       # Refresh rate for the animation in ms



class AttitudeFrame(BaseParamFrame):
    def __init__(self, parent, param=None):
        self.param = TimeParameters() if param is None else param
        super().__init__(parent, self.param)
        self.draw_frame()


    def reset(self):
        self.param = TimeParameters()
        self.update_values(self.param)


    def draw_frame(self):
        self.add_field("Start time (s): ",
                       self.param_vars["t0"])

        self.add_field("End time(s): ",
                       self.param_vars["t_end"])

        self.add_field("Time step (ms): ",
                       self.param_vars["dt"])

        self.add_field("Interval: ",
                       self.param_vars["interval"])




