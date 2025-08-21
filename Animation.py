from re import A
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
from scipy.spatial.transform import Rotation
from functools import partial

import Satellite
import Controller
from parameters import *

class AttitudeAnimation:
    def __init__(self, 
                 sat: Satellite.Satellite,
                 ref: ReferenceParameters,
                 controller: Controller.Controller,
                 parameters_ani=None, 
                 parameters_plt=None) -> None:

        self.sat: Satellite.Satellite = sat
        self.ref: ReferenceParameters = ref
        self.controller: Controller.Controller = controller

        self.load_animation_parameters(parameters_ani)
        self.load_plot_parameters(parameters_plt)

        # self.data = {'time': np.zeros([int((self.parameters_animation.T_stop - 
                                            # self.parameters_animation.T0))])}
        self.data = {'time': np.arange(self.parameters_animation.T0, 
                                       self.parameters_animation.T_stop,
                                       self.parameters_animation.dt)}
        self.data['ref'] = np.zeros_like(self.data['time'])

        self.fig = plt.figure(figsize=(self.parameters_plot.width,
                                       self.parameters_plot.height))
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.scatter(0.0, 0.0, 0.0, color='black')

        # Prevent the animation from starting immediately
        self.animation = FuncAnimation(
            self.fig,
            partial(self.update),
            frames=len(self.data['time']),
            interval=self.parameters_animation.interval,
            init_func=self.init,
            blit=True
        )
        self.fig.tight_layout()
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.pause = True
        self.animation.pause()


    def on_click(self, event):
        self.pause ^=True
        if self.pause:
            self.animation.event_source.start()
            return
        self.animation.event_source.stop()


    def load_animation_parameters(self, parameters=None):
        self.parameters_animation: AnimationParameters = AnimationParameters()                if parameters is None else parameters


    def load_plot_parameters(self, parameters=None):
        self.parameters_plot = PlotParameters() if parameters is \
                                    None else parameters


    def init(self):
        self.axes = []
        self.axes_ref = []
        for i in range(3):
            (line, ) = self.ax.plot(
                                [],
                                [],
                                [],
                                self.parameters_plot.colors[
                                i % len(self.parameters_plot.colors)])
            self.axes.append(line)

            (line_ref, ) = self.ax.plot(
                                [],
                                [],
                                [],
                                self.parameters_plot.colors[
                                i % len(self.parameters_plot.colors)],
                                alpha=self.parameters_plot.alpha)
            self.axes_ref.append(line_ref)

        self.ax.set_xlim((-1.5, 1.5))
        self.ax.set_ylim((-1.5, 1.5))
        self.ax.set_zlim((-1.5, 1.5))

        # Set the labes of the plot
        self.ax.set_xlabel("x-axis")
        self.ax.set_ylabel("y-axis")
        self.ax.set_zlabel("z-axis")

        return self.axes + self.axes_ref


    def draw_ref(self):
        R = self.ref.rot.as_matrix()
        for i in range(3):
            self.axes_ref[i].set_data([0.0, R[0, i]], [0.0, R[1, i]])
            self.axes_ref[i].set_3d_properties([0.0, R[2, i]])


    def draw_satellite(self):
        R = self.sat.state.rot.as_matrix()
        for i in range(3):
            self.axes[i].set_data([0.0, R[0, i]], [0.0, R[1, i]])
            self.axes[i].set_3d_properties([0.0, R[2, i]])


    def update(self, frame):
        L = self.controller.output(self.sat.state, self.ref)
        self.sat.update_attitude(L=L, dt=self.parameters_animation.dt)

        self.draw_satellite()
        if self.parameters_plot.reference:
            self.draw_ref()

        return self.axes + self.axes_ref


    def flush_plot(self):
        for line in self.axes:
            line.set_data([], [])
            line.set_3d_properties([])

        for line in self.axes_ref:
            line.set_data([], [])
            line.set_3d_properties([])


class SimulationFrame(tk.Frame):
    def __init__(self, 
                 parent,
                 anim_obj: AttitudeAnimation):

        super().__init__(parent)
        self.anim_obj = anim_obj

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

        self.running = True


    def start(self):
        if not self.running:
            self.running = True
        self.anim_obj.animation.resume()


    def stop(self):
        if self.running:
            self.running = False
        self.anim_obj.animation.pause()


    def reset(self):
        self.stop()
        self.anim_obj.sat.reset()
        self.anim_obj.flush_plot()
        self.canvas.draw_idle()
        self.anim_obj.init()
        self.anim_obj.draw_satellite()
        if self.anim_obj.parameters_animation.reference:
            self.anim_obj.draw_ref()
        self.canvas.draw_idle()


def test_parameters_sat():
    rot = Rotation.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
    param_sat = Satellite.Satellite_State(
        # rot=rot,
        w=np.zeros(3),
        w_dot=np.zeros(3),
    )
    param_ref = ReferenceParameters(
        rot=rot,
        W=np.zeros(3),
        W_dot=np.zeros(3)
    )

    sat = Satellite.Satellite()
    sat.load_satellite_parameters(param_sat)
    controller = Controller.PDController()
    return sat, param_ref, controller


def create_test_parameters():
    param_ani = AnimationParameters(
        T0=0.0,
        T_stop= 20.0,
        dt= 0.05,
        interval=50,
        animation_file_path="Simulations/animation.gif",
        plot_file_path="Plots/simulation.pdf"
    )

    param_plt = PlotParameters(
        reference=True,
        alpha=0.7,
        width=6,
        height=4,
        colors=["red", "green", "blue"]
    )
    return param_ani, param_plt


def create_test_animation():
    anim = AttitudeAnimation(*test_parameters_sat(), *create_test_parameters())
    return anim


def test_3d_animation():
    anim = create_test_animation()
    print(f"Satellite quaternion: {anim.sat.state.rot.as_quat()}")
    print(f"Reference quaternion: {anim.ref.rot.as_quat()}")
    print("-----------------------------------------------------------------------")
    plt.show()


def close_app(root):
    plt.close("all")
    root.destroy()


def test_page():
    root = tk.Tk()
    root.title("Test of simulation frame")
    sim_page = SimulationFrame(root, create_test_animation())
    sim_page.pack(fill="both", expand=True)
    root.protocol("WM_DELETE_WINDOW", lambda: close_app(root))
    root.mainloop()


if __name__ == '__main__':
    # test_3d_animation()
    test_page()

