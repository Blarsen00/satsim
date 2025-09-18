import tkinter as tk
from tkinter import StringVar, ttk, filedialog

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from collections import defaultdict
from typing import Optional, List, Tuple

from Actuators.actuator import Actuator, PLOTTING_PARAMETES
from Actuators.actuatorSystem import ActuatorSystem
from misc import test_page


class ActuatorSystemFrame(tk.Frame):
    def __init__(self, 
                 parent,
                 actuator_system: Optional[ActuatorSystem]=None):
        super().__init__(parent)
        self.actuator_system = actuator_system if actuator_system is not None else ActuatorSystem()
        # print(actuator_system)
        self.draw_frame()

    def draw_frame(self):
        colors = ["red", "green", "blue", "magenta", "cyan"]
        for i, anim in enumerate(self.actuator_system.animations):
            anim.color = colors[i % len(colors)]

            row = tk.Frame(self)
            row.grid(row=i, column=0, sticky="nsew", padx=1, pady=1)

            # give each row equal "weight" so they share space
            self.rowconfigure(i, weight=1)
            self.columnconfigure(0, weight=1)

            left = tk.Frame(row)
            left.pack(side="left", fill="y", padx=1, pady=1)

            info_label = tk.Label(left, text=f"{i}: {anim.actuator.name} -> ({', '.join([f'{x:.1f}' for x in anim.actuator.axis])} )")
            info_label.pack(side="top", fill="x")

            ref = tk.IntVar(value=1)
            toggle = self.create_ref_toggle(left, var=ref)
            toggle.pack(side="top", fill="x")

            # Create a frame to hold the display values
            values_frame = tk.Frame(left)
            values_frame.pack(side="top", fill="x")

            for key in anim.actuator.data.keys():
                        if key == "time":
                            continue
                        # Create a StringVar and link it to a Label
                        display_var = tk.StringVar()
                        anim.display_vars[key] = display_var
                        tk.Label(values_frame, textvariable=display_var).pack(side="top", fill="x")

            var = StringVar()
            menu = self.create_actuator_menu(left, anim.actuator, var, lambda var: self.swap_plot(var, i))
            menu.pack(side="top", fill="x", padx=1, pady=1)

            right = tk.Frame(row)
            right.pack(side="left", fill="both", expand=True, padx=1, pady=1)

            canvas = FigureCanvasTkAgg(anim.fig, master=right)
            canvas_widget = canvas.get_tk_widget()
            canvas_widget.pack(side="top", fill="both", expand=True)

    @staticmethod
    def create_actuator_menu(frame: tk.Frame,
                             actuator: Actuator,
                             var: tk.StringVar,
                             cb) -> tk.OptionMenu:

        # keys = [key for key in actuator.data.keys()]
        keys = [key for key in actuator.data.keys() if key != "time"]
        menu = tk.OptionMenu(
            frame, 
            var,
            keys[0],
            *keys[1:],
            command=cb)
        return menu

    @staticmethod
    def create_ref_toggle(frame, var: tk.IntVar) -> tk.Checkbutton:
        toggle = tk.Checkbutton(frame, text="ref", variable=var)
        return toggle

    def swap_plot(self, var: StringVar, actuator_idx: int):
        self.actuator_system.animations[actuator_idx].key = var


if __name__ == '__main__':
    root = tk.Tk()
    # test_page(root, SimulationFrame(root))
    test_page(root, ActuatorSystemFrame(root))
