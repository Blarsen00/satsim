import tkinter as tk
import numpy as np

from frames.base_frame import BaseParamFrame

from satellite import Satellite
from simulation import PhysicalState


class SatelliteParamFrame(BaseParamFrame):
    def __init__(self, parent, param=None):
        self.sat: Satellite = Satellite() if param is None else param
        self.param: PhysicalState = self.sat.state
        self.J_vars = [[tk.StringVar(value=str(np.eye(3)[i, j])) for i in range(3)]
                        for j in range(3)]

        super().__init__(parent, param)
        self.draw_frame()

    def draw_frame(self):
        self.add_array_field("Inertia Matrix (J): ", self.J_vars)

    def apply(self):
        J = [[float(self.J_vars[i][j].get()) for j in range(3)]
            for i in range(3)]
        self.sat.J = np.array(J)
        return super().apply()

    def update_values(self, param=None):
        super().update_values(param)
        for i in range(3):
            for j in range(3):
                self.J_vars[i][j].set(str(self.sat.J[i,j]))

