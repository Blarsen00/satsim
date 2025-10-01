import tkinter as tk
from tkinter import filedialog
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from functools import partial
from typing import Optional

from frames.actuatorSystem_frame import ActuatorSystemFrame
from Actuators.actuatorSystem import ActuatorSystem
from attitude import AttitudeAnimation, create_simulation
from animation import Animator
from frames.satellite_frame import SatelliteFrame
from misc import test_page


class SimulationFrame(tk.Frame):
    def __init__(self, 
                 parent,
                 anim_obj: Optional[AttitudeAnimation]=None):

        super().__init__(parent)


        controls = tk.Frame(self)
        controls.pack(side="bottom", fill="x")

        # Main vertical split: top (plots) + bottom (controls)
        top_frame = tk.Frame(self)
        top_frame.pack(side="top", fill="both", expand=True)

        # Left side (attitude simulation)
        # self.anim_obj = AttitudeSimulation() if anim_obj is None else anim_obj
        self.anim_obj = AttitudeAnimation() if anim_obj is None else anim_obj
        self.canvas = FigureCanvasTkAgg(self.anim_obj.fig, master=top_frame)
        self.canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

        self.create_anim()

        right_frame = tk.Frame(top_frame)
        right_frame.pack(side="right", fill="both", anchor="w")

        sat_frame = tk.Frame(right_frame)
        sat_frame.pack(side="top", fill="both", anchor="w")
        self.sat_comp_frame = SatelliteFrame(sat_frame, self.anim_obj)
        self.sat_comp_frame.pack()

        # Right side (actuator plots depreciated. Just using values)
        actuator_frame = tk.Frame(right_frame)
        actuator_frame.pack(side="top", fill="both", expand=True)
        self.act_sys_frame = ActuatorSystemFrame(actuator_frame, self.anim_obj.sat.actuator_system)
        self.act_sys_frame.pack(fill="both", expand=True)
        # ActuatorSystemFrame(actuator_frame, self.anim_obj.sat.actuator_system).pack(fill="both", expand=True)

        # Bottom (control bar)
        # self.animator = Animator(self.anim_obj.sat.actuator_system.animations)
        # self.animator = Animator([self.anim_obj])
        # self.animator = Animator([self.anim_obj] + self.anim_obj.sat.actuator_system.animations)
        self.draw_control_bar(controls)

        self.running = True
        # print(self.anim_obj.sat.state)
        # print(self.anim_obj.ref.state)
        # print(self.anim_obj.controller)

    def create_anim(self):
        self.animation = FuncAnimation(
            self.anim_obj.fig,
            partial(self.update_anim),
            frames=self.anim_obj.t.size,
            interval=self.anim_obj.time.interval,
            init_func=self.init_anim,
            # blit=False
            blit=True
        )
        self.anim_obj.fig.tight_layout()
        self.animation.pause()
        self.pause = True

    def init_anim(self):
        return self.anim_obj.init_anim()

    def update_anim(self, frame):
        output = self.anim_obj.update_anim(frame)
        self.act_sys_frame.update_values()
        self.sat_comp_frame.update_values()
        return output

    def start_anim(self):
        self.pause = False
        self.animation.resume()

    def stop_anim(self):
        self.pause = True
        self.animation.pause()

    def reset(self):
        self.anim_obj.reset_anim()
        self.act_sys_frame.update()

        self.stop_anim()
        self.init_anim()
        self.update_anim(0)
        self.anim_obj.fig.canvas.draw()

    def draw_control_bar(self, frame: tk.Frame):
        self.start_btn = tk.Button(frame, text="Start", command=lambda: self.start_anim())
        # self.start_btn = tk.Button(frame, text="Start", command=self.anim_obj.start_anim)
        # self.start_btn = tk.Button(frame, text="Start", command=self.animator.start)
        self.start_btn.pack(side=tk.LEFT)

        self.stop_btn = tk.Button(frame, text="Stop", command=lambda: self.stop_anim())
        # self.stop_btn = tk.Button(frame, text="Stop", command=self.anim_obj.stop_anim)
        # self.stop_btn = tk.Button(frame, text="Stop", command=self.animator.stop)
        self.stop_btn.pack(side=tk.LEFT)

        self.reset_btn = tk.Button(frame, text="Reset", command=lambda: self.reset())
        # self.reset_btn = tk.Button(frame, text="Reset", command=self.anim_obj.reset_anim)
        # self.reset_btn = tk.Button(frame, text="Reset", command=self.animator.reset)
        self.reset_btn.pack(side=tk.LEFT)

        self.randomize_btn = tk.Button(frame, text="Randomize", command=self.randomize)
        self.randomize_btn.pack(side=tk.LEFT)

        self.reference = tk.IntVar(value=1)
        self.reference_btn = tk.Checkbutton(frame,
                                            text="Reference",
                                            variable=self.reference,
                                            onvalue=1,
                                            offvalue=0,
                                            command=self.set_ref)
        self.reference_btn.pack(side=tk.LEFT)

        self.file_path: str = ""
        self.save_as_btn = tk.Button(frame,
                                  text="Save as",
                                  command=self.save_as)
        self.save_btn = tk.Button(frame,
                                  text="Save",
                                  command=self.save)
        self.save_btn.pack(side=tk.RIGHT)
        self.save_as_btn.pack(side=tk.RIGHT)

    def save_as(self):
        file_path: str = filedialog.asksaveasfilename()
        if file_path == "":
            return

        self.file_path = file_path
        self.save()

    def save(self):
        if self.file_path == "":
            return self.save_as()
        # self.anim_obj.animation.save(self.file_path, 
        self.animation.save(self.file_path, 
                                 writer="pillow",
                                 fps=int(1000/self.anim_obj.time.interval))
                                 # fps=int(1000/self.anim_obj.parameters_animation.interval))


    def set_ref(self):
        self.anim_obj.draw_reference = bool(self.reference.get())
        self.anim_obj.flush_plot()
        self.canvas.draw_idle()
        self.anim_obj.init_anim()
        self.anim_obj.draw_satellite()
        if self.anim_obj.draw_reference:
            self.anim_obj.draw_ref()
        self.canvas.draw_idle()


    def randomize(self):
        self.anim_obj.sat.state.randomize_attitude()
        self.anim_obj.ref.state.randomize_attitude()
        self.reset()
        # self.anim_obj.reset_anim()
        # self.animator.reset()


if __name__ == '__main__':
    root = tk.Tk()
    # test_page(root, SimulationFrame(root))
    test_page(root, SimulationFrame(root, create_simulation()))
