import tkinter as tk
import matplotlib.pyplot as plt

# Calculate
from attitude import AttitudeSimulation, create_simulation
from attitude_frame import TimeParameters
from controller import Controller, PDController
from reference import BaseReference
from satellite import Satellite

# Frames
from base_frame import BaseParamFrame
from simulation import PhysicalState
from simulation_frame import SimulationFrame
from reference_frame import ReferenceFrame
from controller_frame import ControllerFrame
from plotting_frame import AniParamFrame


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.geometry("1000x800")
        self.title("Satellite Attitude Simulation")
        self.protocol('WM_DELETE_WINDOW', self.close_app)

        self.menu_bar = tk.Frame(self, bg="lightgray")
        self.menu_bar.pack(side=tk.TOP, fill="x")

        self.container = tk.Frame(self, bg="white")
        self.container.pack(fill="both", expand=True)

        self.ani_obj = create_simulation()
        self.frames = {"Simulation": SimulationFrame(self.container, self.ani_obj),
                       "Controller": ControllerFrame(self.container),
                       "Reference": ReferenceFrame(self.container, PhysicalState()),
                       "Params" : AniParamFrame(self.container, TimeParameters())}
        # self.frames = {"Controller": ControllerFrame(self.container)}
        # self.frames = {"Reference": ReferenceFrame(self.container)}
        # self.frames = {"Simulation": SimulationFrame(self.container, self.ani_obj)}
        # self.frames = {"Params" : AniParamFrame(self.container)}

        self.init_menu()
        self.active_frame: tk.Frame = self.frames["Simulation"]
        self.active_frame.pack(side=tk.TOP, fill="both", expand=True)


    def init_menu(self):
        tk.Button(self.menu_bar, 
                  text="Simulation",
                  command=lambda: self.show_frame("Simulation")).pack(side=tk.LEFT)
        tk.Button(self.menu_bar,
                  text="Animation",
                  command=lambda: self.show_frame("Params")).pack(side=tk.LEFT)
        tk.Button(self.menu_bar,
                  text="Controller",
                  command=lambda: self.show_frame("Controller")).pack(side=tk.LEFT)
        tk.Button(self.menu_bar,
                  text="Reference",
                  command=lambda: self.show_frame("Reference")).pack(side=tk.LEFT)


    def close_app(self):
        plt.close("all")
        self.destroy()


    def show_frame(self, page_name):
        self.active_frame.pack_forget()
        self.active_frame = self.frames[page_name]
        self.active_frame.pack(side=tk.TOP, fill="both", expand=True)
        self.update_params()


    def update_params(self):
        self.ani_obj.load_animation_parameters(self.frames["Params"].param)
        self.ani_obj.controller = self.frames["Controller"].get_controller()
        print(f"Selected Controller: {self.ani_obj.controller.param}")


if __name__ == "__main__":
    app = App()
    app.mainloop()
    app.quit()
