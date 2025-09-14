import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation

from functools import partial
from typing import Optional

from frames.attitude_frame import TimeParameters, PlotParameters
from satellite import Satellite
from reference import BaseReference
from controller import Controller, PDController, SMCController
from simulation import PhysicalState, Simulate


class AttitudeSimulation:
    def __init__(self, 
                 sat:Optional[Satellite]=None,
                 ref: Optional[BaseReference]=None,
                 controller: Optional[Controller]=None,
                 parameters_ani=None,
                 parameters_plt=None) -> None:

        self.sat: Satellite = Satellite() if sat is None else sat
        self.ref: BaseReference = BaseReference() if ref is None else ref
        self.controller: Controller = PDController() if controller is None else controller
        self.parameters_animation: TimeParameters = TimeParameters() if parameters_ani is None else parameters_ani
        self.parameters_plot = PlotParameters() if parameters_plt is None else parameters_plt

        self.data = {'time': np.arange(self.parameters_animation.t0, 
                                       self.parameters_animation.t_end,
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
        if parameters is None:
            self.parameters_animation: TimeParameters = TimeParameters() 
        else:
            self.parameters_animation: TimeParameters = parameters


    def load_plot_parameters(self, parameters=None):
        self.parameters_plot = PlotParameters() if parameters is \
                                    None else parameters


    def load_controller_parameters(self, parameters=None):
        self.controller.load_params(parameters)


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

        self.ax.set_xlim((-1.0, 1.0))
        self.ax.set_ylim((-1.0, 1.0))
        self.ax.set_zlim((-1.0, 1.0))

        # Set the labes of the plot
        self.ax.set_xlabel("x-axis")
        self.ax.set_ylabel("y-axis")
        self.ax.set_zlabel("z-axis")

        return self.axes + self.axes_ref


    def draw_ref(self):
        R = self.ref.state.rot.as_matrix()
        for i in range(3):
            self.axes_ref[i].set_data([0.0, R[0, i]], [0.0, R[1, i]])
            self.axes_ref[i].set_3d_properties([0.0, R[2, i]])


    def draw_satellite(self):
        R = self.sat.state.rot.as_matrix()
        for i in range(3):
            self.axes[i].set_data([0.0, R[0, i]], [0.0, R[1, i]])
            self.axes[i].set_3d_properties([0.0, R[2, i]])


    def update(self, frame):
        # Calculate
        L = self.controller.output(self.sat.state, self.ref.state, J=self.sat.J)
        self.sat.state = Simulate.update_state(L,
                                               self.sat.state,
                                               self.sat.J,
                                               self.parameters_animation.dt)
        self.ref.update(self.parameters_animation.dt)

        # Draw
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


def test_parameters_sat():
    rot = Rotation.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
    sat_param = PhysicalState(
        rot=rot,
        w=np.zeros(3),
        w_dot=np.zeros(3),
    )
    ref_param = PhysicalState(
        rot=rot,
        # w=np.zeros(3),
        w=np.ones(3) * 1.2,
        w_dot=np.zeros(3)
    )

    ref = BaseReference()
    ref.state = ref_param

    sat = Satellite()
    sat.load_satellite_parameters(sat_param)

    # controller = PDController()
    controller = SMCController()
    return sat, ref, controller


def create_test_parameters():
    param_ani = TimeParameters(
        t0=0.0,
        t_end= 20.0,
        dt= 0.05,
        interval=50,
    )

    param_plt = PlotParameters(
        reference=True,
        alpha=0.7,
        width=6,
        height=4,
        colors=["red", "green", "blue"]
    )
    return param_ani, param_plt


def create_simulation():
    anim = AttitudeSimulation(*test_parameters_sat(), *create_test_parameters())
    return anim


def test_animation():
    anim = create_simulation()
    print(f"Satellite quaternion: {anim.sat.state.rot.as_quat()}")
    print(f"Reference quaternion: {anim.ref.state.rot.as_quat()}")
    print("-----------------------------------------------------------------------")
    plt.show()


if __name__ == '__main__':
    test_animation()

