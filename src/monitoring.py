import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.animation import FuncAnimation
from functools import partial

from satellite import Satellite
from frames.attitude_frame import TimeParameters
from simulation import Simulate, PhysicalState
from controller import Controller, PDController, SMCController
from reference import BaseReference


class Monitor:
    def __init__(self) -> None:
        self.data = {}
        self.fig = plt.figure(figsize=(6, 4))

        self.sat: Satellite = Satellite()
        self.ref: BaseReference = BaseReference()
        self.cnt: Controller = PDController()

        self.H = np.zeros((3,3))

        self.T = TimeParameters()
        T_arr = np.arange(self.T.t0, self.T.t_end, step=self.T.dt)

        self.data["time"] = T_arr


    def simulate(self):
        for t in self.data["time"]:
            L = self.cnt.output(self.sat.state,
                                self.sat.state,
                                J=self.sat.J)

            H = Simulate.calculate_angular_momentum(L,
                                                    self.sat.state.w,
                                                    self.H,
                                                    self.T.dt)

            state = Simulate.update_state(L,
                                          self.sat.state,
                                          self.sat.J,
                                          self.T.dt)

            self.log_data("L", L)
            self.log_data("sat", state)
            self.log_data("H", H)


    def log_data(self, key: str, value):
        self.data[key] = self.data.get(key, []) + value


    @staticmethod
    def add_to_ax(ax: Axes, key):
        pass

    def make_torque():
        pass



if __name__ == "__main__":
    pass

