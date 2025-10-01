import tkinter as tk
from dataclasses import asdict
from typing import Optional, Any

from frames.attitude_frame import TimeParameters
from frames.base_frame import BaseParamFrame

from misc import test_page


class AniParamFrame(BaseParamFrame):
    def __init__(self, parent, time_param: Optional[TimeParameters]=None):
        self.time_param: TimeParameters = TimeParameters() if time_param is None else time_param
        self.param = asdict(self.time_param)
        super().__init__(parent, self.param)

    def reset(self):
        self.time_param = TimeParameters()
        self.param = asdict(self.time_param)
        self.update_values(self.param)

    def apply(self):
        super().apply()
        self.time_param = TimeParameters(**self.param)

    def get_obj(self) -> Any:
        return self.time_param

    def draw_frame(self):
        self.add_field("Start time (s): ", self.vars["t0"])
        self.add_field("End time (s): ", self.vars["t_end"])
        self.add_field("Simulation dt (s): ", self.vars["dt"])
        self.add_field("Refresh rate (ms): ", self.vars["interval"])
        return super().draw_frame()


if __name__ == '__main__':
    root = tk.Tk()
    test_page(root, AniParamFrame(root))
