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
from animation import BaseAnimation

class AttitudeAnimation(BaseAnimation):
    def __init__(self, 
                 sat:Optional[Satellite]=None,
                 ref: Optional[BaseReference]=None,
                 controller: Optional[Controller]=None) -> None:

        self.sat: Satellite = Satellite() if sat is None else sat
        self.ref: BaseReference = BaseReference() if ref is None else ref
        self.controller: Controller = PDController() if controller is None else controller
        # print(self.controller)

        # Plotting axes
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.scatter(0.0, 0.0, 0.0, color='black')

        self.colors = ["red", "green", "blue"]
        self.alpha = 0.6
        self.draw_reference = True

        super().__init__()

    def load_controller_parameters(self, parameters=None):
        self.controller.load_params(parameters)

    def init_anim(self):
        self.axes = []
        self.axes_ref = []
        for i in range(3):
            (line, ) = self.ax.plot(
                                [],
                                [],
                                [],
                                self.colors[i % len(self.colors)])
            self.axes.append(line)

            (line_ref, ) = self.ax.plot(
                                [],
                                [],
                                [],
                                self.colors[
                                i % len(self.colors)],
                                alpha=self.alpha)
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

    def draw(self):
        self.draw_satellite()
        if self.draw_reference:
            self.draw_ref()
        return super().draw()

    def update_anim(self, frame, dt=0.1):
        # Calculate
        u = self.controller.output(self.sat.state, self.ref.state, J=self.sat.J)
        L_applied = self.sat.actuator_system.apply_torque(u,
                                                          b0=self.sat.b0,
                                                          R=self.sat.state.rot.as_matrix(),
                                                          kp=2.0,
                                                          w=self.sat.state.w)
        # print(L)
        # print(L, L_applied)
        self.sat.state = Simulate.update_state(L_applied,
        # self.sat.state = Simulate.update_state(L,
                                               self.sat.state,
                                               self.sat.J,
                                               # dt)
                                               self.time.dt)
        # self.time.dt
        self.ref.update(self.time.dt)
        # self.ref.update(dt)

        # Draw
        self.draw()
        return self.axes + self.axes_ref

    def reset_anim(self):
        self.flush_plot()
        self.sat.reset()
        # super().reset_anim()

    def flush_plot(self):
        for line in self.axes:
            line.set_data([], [])
            line.set_3d_properties([])

        for line in self.axes_ref:
            line.set_data([], [])
            line.set_3d_properties([])

def test_parameters_sat():
    rot = Rotation.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
    rot_I = Rotation.from_quat([0, 0, 0, 1.0])
    sat_param = PhysicalState(
        rot=rot_I,
        w=np.zeros(3),
        w_dot=np.zeros(3),
    )
    ref_param = PhysicalState(
        rot=rot,
        w=np.zeros(3),
        # w=np.ones(3) * 0.01,
        w_dot=np.zeros(3)
    )

    ref = BaseReference()
    ref.state = ref_param

    sat = Satellite()
    sat.load_satellite_parameters(sat_param)

    controller = PDController()
    # controller = SMCController()
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
    # anim = AttitudeSimulation(*test_parameters_sat(), *create_test_parameters())
    anim = AttitudeAnimation(*test_parameters_sat())
    return anim


def test_animation():
    anim = create_simulation()
    print(f"Satellite quaternion: {anim.sat.state.rot.as_quat()}")
    print(f"Reference quaternion: {anim.ref.state.rot.as_quat()}")
    print("-----------------------------------------------------------------------")
    plt.show()


if __name__ == '__main__':
    test_animation()

