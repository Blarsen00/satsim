import tkinter as tk
from tkinter import ttk, filedialog

from dataclasses import asdict, dataclass, field
from frames.base_frame import BaseParamFrame
from animation import TimeParameters

from typing import List, Optional, Any

from misc import test_page


@dataclass 
class PlotParameters:
    width: int = 6          # Inches
    height: int = 4         # Inches
    alpha: float = 0.7      # alpha value for references
    reference: bool = True
    colors: List[str] = field(default_factory=lambda: ["red", "blue", "green"])


class AttitudeFrame(BaseParamFrame):
    def __init__(self,
                 parent,
                 time_param:Optional[TimeParameters]=None):
        self.time_param = TimeParameters() if time_param is None else time_param
        self.param = asdict(self.time_param)
        super().__init__(parent, self.param)

    def reset(self):
        self.time_param = TimeParameters()
        self.update_values(self.param)

    def draw_frame(self):
        self.add_field("Start time (s): ", self.vars["t0"])
        self.add_field("End time(s): ", self.vars["t_end"])
        self.add_field("Time step (ms): ", self.vars["dt"])
        self.add_field("Interval: ", self.vars["interval"])

    def apply(self):
        super().apply()
        self.time_param = TimeParameters(**self.param)

    def get_obj(self) -> Any:
        return self.time_param


if __name__ == '__main__':
    root = tk.Tk()
    test_page(root, AttitudeFrame(root))

