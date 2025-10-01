from copy import deepcopy
import tkinter as tk
from typing import Dict, Optional, Any, Type, Union
from Actuators.actuator import Actuator
from frames.base_frame import BaseParamFrame
from Actuators.reactionwheel import ReactionWheel, make_reactionwheel_plot
from misc import test_page
from scipy.spatial.transform import Rotation

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class ReactionWheelFrame(BaseParamFrame):
    def __init__(self,
                 parent,
                 rw: Optional[ReactionWheel]=None):
        self.rw = ReactionWheel() if rw is None else rw
        self.original = ReactionWheel() if rw is None else rw

        param = {key: getattr(self.rw, key) for key in self.rw._params}
        self.main_param = {key: getattr(self.rw, key) for key in self.rw._main_params.keys()}
        super().__init__(parent, param)

    def draw_concise_frame(self):
        for key, val in self.rw._main_params.items():
            if isinstance(getattr(self.rw, key), list) or \
                    isinstance(getattr(self.rw, key), np.ndarray) or \
                    isinstance(getattr(self.rw, key), Rotation):
                self.add_array_field(val, self.vars[key])
            elif isinstance(getattr(self.rw, key), float) or \
                    isinstance(getattr(self.rw, key), int) or \
                isinstance(getattr(self.rw, key), np.floating):
                self.add_field(val, self.vars[key])

    def draw_frame(self):
        params = tk.Frame(self.canvas)
        params.pack(side="left",
                    fill="x",
                    pady=self.base_padding_y,
                    padx=self.base_padding_x)

        self.add_divider(frame=params)
        self.add_list_field("Axis (x, y, z): ", self.vars["axis"], frame=params)
        self.add_field("Anuglar rate, w (rad/s)", self.vars["w"], frame=params)
        self.add_divider(frame=params)

        self.add_array_field("J: ", self.vars["J"], frame=params)
        self.add_divider(frame=params)

        self.add_field("Max current I: ", self.vars["max_I"], frame=params)
        self.add_field("Motor constant Km", self.vars["km"], frame=params)
        self.add_divider(frame=params)

        self.add_field("Coloumb damping coefficient (dc): ", self.vars["dc"], frame=params)
        self.add_field("Viscous damping coefficient (dv): ", self.vars["dv"], frame=params)

        self.plot_frame = tk.Frame(self.canvas)
        self.plot_frame.pack(side="left",
                        fill="x",
                        pady=self.base_padding_y,
                        padx=self.base_padding_x)
        self.rw_plot = make_reactionwheel_plot(self.rw)
        self.rw_canvas = FigureCanvasTkAgg(self.rw_plot, master=self.plot_frame)
        self.rw_canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

    def reset(self):
        self.rw = deepcopy(self.original)
        self.param = {key: getattr(self.rw, key) for key in self.rw._params}
        return super().reset()

    def apply(self):
        super().apply()
        for key in self.param.keys():
            setattr(self.rw, key, self.param[key])
        self.redraw()
        print(self.rw)

    def get_obj(self) -> Any:
        return self.rw


def create_actuator_frame(parent: tk.Widget, actuator: Optional[Union[Type[Actuator], Actuator]]) -> tk.Frame:
    """ Creates and returns the appropriate parameter frame for the given actuator object.
    """
    actuator_class = type(actuator)
    # FrameClass: Type[BaseParamFrame] = ACTUATOR_FRAME_MAP.get(actuator_class)
    FrameClass: Type[BaseParamFrame] = ACTUATOR_FRAME_MAP[type(actuator)]
    print(FrameClass)
    if FrameClass is None:
        raise ValueError(
            f"No parameter frame registered for actuator type: {actuator_class.__name__}"
        )
    return FrameClass(parent, actuator)


""" Mapping from actuator class to its corresponding parameter frame.
"""
ACTUATOR_FRAME_MAP: Dict[Type[Actuator], Type[BaseParamFrame]] = {
    ReactionWheel: ReactionWheelFrame
}

if __name__ == '__main__':
    rw = ReactionWheel(axis=np.array([0.0, 0.0, 1.0]))
    root = tk.Tk()
    test_page(root, ReactionWheelFrame(root))

    root = tk.Tk()
    test_page(root, create_actuator_frame(root, rw))
