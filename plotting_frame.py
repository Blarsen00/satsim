import tkinter as tk

from attitude_frame import TimeParameters
from base_frame import BaseParamFrame
from misc import test_page


class AniParamFrame(BaseParamFrame):
    def __init__(self, parent, param=None):
        self.param = TimeParameters() if param is None else param
        super().__init__(parent, self.param)
        self.draw_frame()


    def reset(self):
        self.param = TimeParameters()
        self.update_values(self.param)


    def draw_frame(self):
        self.add_field("Start time (s): ", self.param_vars["t0"])
        self.add_field("End time (s): ", self.param_vars["t_end"])
        self.add_field("Simulation dt (s): ", self.param_vars["dt"])
        self.add_field("Refresh rate (ms): ", self.param_vars["interval"])
        return super().draw_frame()


if __name__ == '__main__':
    root = tk.Tk()
    test_page(root, AniParamFrame(root))
