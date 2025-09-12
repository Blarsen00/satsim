import tkinter as tk

from scipy.spatial.transform import Rotation
from reference import BaseReference
from simulation import PhysicalState
from misc import test_page

from frames.base_frame import BaseParamFrame
from frames.satellite_frame import SatelliteParamFrame

from typing import Dict

class ReferenceFrame(BaseParamFrame):
    def __init__(self, parent, param=None):
        self.param = PhysicalState() if param is None else param
        super().__init__(parent, self.param)

        self.q_vars = [tk.StringVar(value=str(self.param.rot.as_quat()[i]))
                        for i in range(4)]
        self.w_vars = [tk.StringVar(value=str(self.param.w[i]))
                       for i in range(3)]

        self.draw_frame()


    def reset(self):
        self.param = PhysicalState()
        self.update_values(self.param)


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

        self.add_list_field("Quaternion: ", self.q_vars)
        self.add_list_field("Angular Velocity: ", self.w_vars)


    def apply(self):
        q = [float(self.q_vars[i].get()) for i in range(4)]
        w = [float(self.w_vars[i].get()) for i in range(3)]
        setattr(self.param, "rot", Rotation.from_quat(q))
        setattr(self.param, "w", w)
        self.update_values()
        return super().apply()


    def update_values(self, param=None):
        super().update_values(param)

        quat = self.param.rot.as_quat()
        w = self.param.w

        for i in range(4):
            self.q_vars[i].set(f"{quat[i]:.3f}")

        for i in range(3):
            self.w_vars[i].set(f"{w[i]:.3f}")


    def get_reference(self):
        return BaseReference.get_reference(self.param)


# class SettingsFrame(tk.Frame):
#     def __init__(self, parent):
#         super().__init__(parent)
#
#         self.frames: Dict[str, BaseParamFrame] = {
#             "Reference": ReferenceFrame(self),
#             "Satellite": SatelliteParamFrame(self)
#         }
#         for frame in self.frames:
#             frame.draw_frame()



if __name__ == '__main__':
    root = tk.Tk()
    test_page(root, ReferenceFrame(root))


