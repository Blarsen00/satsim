from dataclasses import asdict
import tkinter as tk
from typing import Optional, Any
import numpy as np
from attitude import AttitudeAnimation
from misc import test_page

from frames.base_frame import BaseParamFrame

from satellite import Satellite
from simulation import PhysicalState


class SatelliteParamFrame(BaseParamFrame):
    def __init__(self, parent, sat:Optional[Satellite]=None):
        self.sat: Satellite = Satellite() if sat is None else sat

        self.param = asdict(self.sat.state)
        self.param["J"] = self.sat.J
        super().__init__(parent, self.param)

    def draw_frame(self):
        row = tk.Frame(self.canvas)
        row.pack(side="top", fill="x")
        tk.Label(row, text="State").pack(side="top",
                                         fill="x",
                                         padx=self.base_padding_x,
                                         pady=self.base_padding_y)
        self.add_divider(frame=self.canvas)
        self.add_list_field("Attitude (q): ", self.vars["rot"])
        self.add_list_field("Angular rate (rad/s): ", self.vars["w"])
        self.add_divider(frame=self.canvas)
        self.add_array_field("Inertia Matrix (J): ", self.vars["J"])

    def reset(self):
        self.sat = Satellite()
        self.param = asdict(self.sat.state)
        self.param["J"] = self.sat.J
        super().reset()

    def apply(self):
        super().apply()
        self.sat.J = self.param["J"]
        self.sat.state = PhysicalState(**{key: value for key, value 
                                        in self.param.items() if key != "J"})
        print(self.sat)

    def get_obj(self) -> Any:
        return self.sat


class SatelliteFrame(tk.Frame):
    def __init__(self, parent, anim:Optional[AttitudeAnimation]=None):
        super().__init__(parent)

        self.anim: AttitudeAnimation = AttitudeAnimation() if anim is None else anim
        self.sat_vars = {
            "rot": [tk.StringVar(value=str(x)) for x in self.anim.sat.state.rot.as_quat()],
            "w": [tk.StringVar(value=str(x)) for x in self.anim.sat.state.w],
        }
        self.ref_vars = {
            "rot": [tk.StringVar(value=str(x)) for x in self.anim.ref.state.rot.as_quat()],
            "w": [tk.StringVar(value=str(x)) for x in self.anim.ref.state.w],
        }
        self.draw_frame()


    def draw_frame(self, frame: Optional[tk.Frame]=None):
        frame = self if frame is None else frame
        width = 12
        bg_ref = "green"
        bg_sat = "blue"

        # Attitude
        tk.Label(frame, text="Attitude").grid(row=0, column=0, columnspan=5)
        tk.Label(frame, text="Satellite: ", anchor="w").grid(row=1, column=0)
        for i in range(4):
            tk.Label(frame,
                     textvariable=self.sat_vars["rot"][i],
                     width=width,
                     bg=bg_sat).grid(row=1, column=i+1)

        tk.Label(frame, text="Reference: ", anchor="w").grid(row=2, column=0)
        for i in range(4):
            tk.Label(frame,
                     textvariable=self.ref_vars["rot"][i],
                     width=width,
                     bg=bg_ref).grid(row=2, column=i+1)

        # Added vertical spacing
        tk.Label(frame, text=" ").grid(row=3, column=0)

        # Angular rate
        tk.Label(frame,
                 text="Angular rate (rad/s)",
                 anchor="w").grid(row=4,
                                  column=0,
                                  columnspan=5)

        tk.Label(frame, text="Satellite: ", anchor="w").grid(row=5, column=0)
        for i in range(3):
            tk.Label(frame,
                     textvariable=self.sat_vars["w"][i],
                     width=width,
                     bg=bg_sat).grid(row=5, column=i+1)

        tk.Label(frame, text="Reference: ", anchor="w").grid(row=6, column=0)
        for i in range(3):
            tk.Label(frame,
                     textvariable=self.ref_vars["w"][i],
                     width=width,
                     bg=bg_ref).grid(row=6, column=i+1)

    def update_values(self):
        """ Feels hacky as all hell, but it works
        """
        for i in range(4):
            sat_q_txt = "{:.3f}".format(self.anim.sat.state.rot.as_quat()[i])
            ref_q_txt = "{:.3f}".format(self.anim.ref.state.rot.as_quat()[i])
            self.sat_vars["rot"][i].set(sat_q_txt)
            self.ref_vars["rot"][i].set(ref_q_txt)
            if i == 3:
                continue
            sat_w_txt = "{:.3f}".format(self.anim.sat.state.w[i])
            ref_w_txt = "{:.3f}".format(self.anim.ref.state.w[i])
            self.sat_vars["w"][i].set(sat_w_txt)
            self.ref_vars["w"][i].set(ref_w_txt)


if __name__ == '__main__':
    # root = tk.Tk()
    # test_page(root, SatelliteParamFrame(root))

    root = tk.Tk()
    test_page(root, SatelliteFrame(root))

