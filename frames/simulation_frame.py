import tkinter as tk
from tkinter import filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from typing import Optional

from attitude import AttitudeSimulation, create_simulation
from misc import test_page


class SimulationFrame(tk.Frame):
    def __init__(self, 
                 parent,
                 anim_obj: Optional[AttitudeSimulation]=None):

        super().__init__(parent)
        self.anim_obj = AttitudeSimulation() if anim_obj is None else anim_obj

        self.canvas = FigureCanvasTkAgg(self.anim_obj.fig, master=self)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        controls = tk.Frame(self)
        controls.pack(side=tk.BOTTOM, fill=tk.X)

        self.start_btn = tk.Button(controls, text="Start", command=self.start)
        self.start_btn.pack(side=tk.LEFT)

        self.stop_btn = tk.Button(controls, text="Stop", command=self.stop)
        self.stop_btn.pack(side=tk.LEFT)

        self.reset_btn = tk.Button(controls, text="Reset", command=self.reset)
        self.reset_btn.pack(side=tk.LEFT)

        self.randomize_btn = tk.Button(controls, text="Randomize", command=self.randomize)
        self.randomize_btn.pack(side=tk.LEFT)

        self.reference = tk.IntVar(value=1)
        self.reference_btn = tk.Checkbutton(controls,
                                            text="Reference",
                                            variable=self.reference,
                                            onvalue=1,
                                            offvalue=0,
                                            command=self.set_ref)
        self.reference_btn.pack(side=tk.LEFT)

        self.file_path: str = ""
        self.save_as_btn = tk.Button(controls,
                                  text="Save as",
                                  command=self.save_as)
        self.save_btn = tk.Button(controls,
                                  text="Save",
                                  command=self.save)
        self.save_btn.pack(side=tk.RIGHT)
        self.save_as_btn.pack(side=tk.RIGHT)

        self.running = True



    def save_as(self):
        file_path: str = filedialog.asksaveasfilename()
        if file_path == "":
            return

        self.file_path = file_path
        self.save()

    def save(self):
        if self.file_path == "":
            return self.save_as()
        self.anim_obj.animation.save(self.file_path, 
                                     writer="pillow",
                                     fps=int(1000/self.anim_obj.parameters_animation.interval))


    def set_ref(self):
        self.anim_obj.parameters_plot.reference = bool(self.reference.get())
        self.anim_obj.flush_plot()
        self.canvas.draw_idle()
        self.anim_obj.init()
        self.anim_obj.draw_satellite()
        if self.anim_obj.parameters_plot.reference:
            self.anim_obj.draw_ref()
        self.canvas.draw_idle()


    def start(self):
        if not self.running:
            self.running = True
        self.anim_obj.animation.resume()


    def stop(self):
        if self.running:
            self.running = False
        self.anim_obj.animation.pause()


    def randomize(self):
        self.anim_obj.sat.state.randomize_attitude()
        self.anim_obj.ref.state.randomize_attitude()
        self.reset()


    def reset(self):
        self.stop()
        self.anim_obj.sat.reset()
        self.anim_obj.flush_plot()
        self.canvas.draw_idle()
        self.anim_obj.init()
        self.anim_obj.draw_satellite()
        if self.anim_obj.parameters_plot.reference:
            self.anim_obj.draw_ref()
        self.canvas.draw_idle()


if __name__ == '__main__':
    root = tk.Tk()
    # test_page(root, SimulationFrame(root))
    test_page(root, SimulationFrame(root, create_simulation()))
