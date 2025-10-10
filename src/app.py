import tkinter as tk
from typing import Optional
import matplotlib.pyplot as plt

# Simulation
from Actuators.actuatorSystem import ActuatorSystem
from attitude import AttitudeAnimation, create_simulation
from controller import Controller, PDController
from frames.actuatorSystem_frame import ActuatorSystemFrame, ActuatorSystemParameterFrame
from frames.satellite_frame import SatelliteParamFrame
from reference import BaseReference
from simulation import PhysicalState
# from controller import Controller, PDController
# from reference import BaseReference
# from satellite import Satellite

# Frames
from frames.base_frame import BaseParamFrame
from frames.attitude_frame import TimeParameters
from frames.simulation_frame import SimulationFrame
from frames.reference_frame import ReferenceFrame
from frames.controller_frame import ControllerFrame
from frames.plotting_frame import AniParamFrame


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.geometry("1400x800")
        self.title("Satellite Attitude Simulation")
        self.protocol('WM_DELETE_WINDOW', self.close_app)

        self.menu_bar = tk.Frame(self, bg="lightgray")
        self.menu_bar.pack(side=tk.TOP, fill="x")

        self.container = tk.Frame(self, bg="white")
        self.container.pack(fill="both", expand=True)

        self.ani_obj: AttitudeAnimation = create_simulation()
        print(self.ani_obj.sat.actuator_system)
        self.frames = {"Simulation": SimulationFrame(self.container, self.ani_obj),
                       # "Control": ControlFrame(self.container),
                       "Controller": ControllerFrame(self.container),
                       "Reference": ReferenceFrame(self.container, self.ani_obj.ref),
                       "Satellite": SatelliteParamFrame(self.container, self.ani_obj.sat),
                       "Actuators": ActuatorSystemParameterFrame(self.container, self.ani_obj.sat.actuator_system),
                       "Params" : AniParamFrame(self.container, TimeParameters())}

        self.init_menu()
        self.active_frame: tk.Frame = self.frames["Simulation"]
        self.active_frame.pack(side=tk.TOP, fill="both", expand=True)
        self.frames["Simulation"].reset()

    def init_menu(self):
        tk.Button(self.menu_bar, 
                  text="Simulation",
                  command=lambda: self.show_frame("Simulation")).pack(side=tk.LEFT)
        # tk.Button(self.menu_bar,
        #           text="Control",
        #           command=lambda: self.show_frame("Control")).pack(side=tk.LEFT)
        tk.Button(self.menu_bar,
                  text="Controller",
                  command=lambda: self.show_frame("Controller")).pack(side=tk.LEFT)
        tk.Button(self.menu_bar,
                  text="Reference",
                  command=lambda: self.show_frame("Reference")).pack(side=tk.LEFT)
        tk.Button(self.menu_bar,
                  text="Satellite",
                  command=lambda: self.show_frame("Satellite")).pack(side=tk.LEFT)
        tk.Button(self.menu_bar,
                  text="Actuators",
                  command=lambda: self.show_frame("Actuators")).pack(side=tk.LEFT)
        tk.Button(self.menu_bar,
                  text="Time",
                  command=lambda: self.show_frame("Params")).pack(side=tk.LEFT)

    def close_app(self):
        plt.close("all")
        self.destroy()

    def show_frame(self, page_name):
        self.active_frame.pack_forget()
        self.active_frame = self.frames[page_name]
        self.active_frame.pack(side=tk.TOP, fill="both", expand=True)
        self.update_params()

    def update_params(self):
        self.ani_obj.controller = self.frames["Controller"].get_controller()
        self.ani_obj.time = self.frames["Params"].get_obj()
        self.ani_obj.ref = self.frames["Reference"].get_obj()

        # Rebuild the satellite object
        self.ani_obj.sat = self.frames["Satellite"].get_obj()
        self.ani_obj.sat.actuator_system = self.frames["Actuators"].get_obj()

        # self.frames["Simulation"].redraw()


class ControlFrame(tk.Frame):
    # NOTE: It honestly not as good as just using dedicated pages for each of them
    """ Joint frame for controlling the following frames: 
            - Controller frame
            - Reference frame
            - Actuator system frame
    """
    def __init__(self, parent):
        super().__init__(parent)
        self.draw_frame()

    def draw_frame(self):
        control = tk.Frame(self)
        reference = tk.Frame(self)
        act = tk.Frame(self)

        control.grid(row=0, column=0)
        reference.grid(row=1, column=0)
        act.grid(row=2, column=0)

        self.controller_frame = ControllerFrame(control)
        self.reference_frame = ReferenceFrame(reference)
        self.actuator_sys_frame = ActuatorSystemParameterFrame(act)

        self.controller_frame.pack()
        self.reference_frame.pack()
        self.actuator_sys_frame.pack()


if __name__ == "__main__":
    app = App()
    app.mainloop()
    app.quit()
