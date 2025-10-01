import tkinter as tk

from scipy.spatial.transform import Rotation
from reference import BaseReference
from simulation import PhysicalState
from misc import test_page

from frames.base_frame import BaseParamFrame
from frames.satellite_frame import SatelliteParamFrame

from typing import Dict, Optional, Any
from dataclasses import fields, asdict

class ReferenceFrame(BaseParamFrame):
    def __init__(self, parent, reference: Optional[BaseReference]=None):
        self.reference = BaseReference() if reference is None else reference
        self.param = asdict(self.reference.state)
        super().__init__(parent, self.param)

    def reset(self):
        self.reference = BaseReference()
        self.param = asdict(self.reference.state)
        return super().reset()

    def apply(self):
        super().apply()
        print(self.param)
        self.reference.state = PhysicalState(**self.param)

    def get_obj(self) -> Any:
        return self.reference

    def draw_frame(self):
        row = tk.Frame(self.canvas)
        row.pack(side=tk.TOP, fill="x")

        tk.Label(row, text="",
                 width=self.base_width).pack(side=tk.LEFT,
                                             padx=self.base_padding_x,
                                             pady=self.base_padding_y)
        labels = ["x", "y", "z", "w"]
        for label in labels:
            tk.Label(row, text=label,
                     width=self.base_entry_width).pack(side=tk.LEFT,
                                                 padx=self.base_padding_x,
                                                 pady=self.base_padding_y)

        self.add_list_field("Quaternion: ", self.vars["rot"])
        self.add_list_field("Angular Velocity: ", self.vars["w"])


if __name__ == '__main__':
    root = tk.Tk()
    test_page(root, ReferenceFrame(root))


