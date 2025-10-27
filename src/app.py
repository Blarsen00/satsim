import tkinter as tk
from typing import Dict, Any
import matplotlib.pyplot as plt

# Simulation
from attitude import AttitudeAnimation, create_simulation
from frames.actuatorSystem_frame import ActuatorSystemParameterFrame
from frames.satellite_frame import SatelliteParamFrame

# Frames
from frames.time_frame import TimeParameters
from frames.simulation_frame import SimulationFrame
from frames.reference_frame import ReferenceFrame
from frames.controller_frame import ControllerFrame
from frames.time_frame import TimeParamFrame


class App(tk.Tk):
    """
    The main application window for the Satellite Attitude Simulation GUI.

    This class inherits from :class:`tkinter.Tk` and manages the application's layout, 
    navigation (using a menu bar), and state synchronization between the user 
    interface frames and the core simulation object.

    Attributes
    ----------
    menu_bar : :class:`tkinter.Frame`
        The top frame containing the navigation buttons.
    container : :class:`tkinter.Frame`
        The main area frame where different content pages (frames) are packed.
    ani_obj : :class:`simulation_modules.AttitudeAnimation`
        The core simulation object containing the satellite, reference, controller, 
        and time parameters. Initialized via ``create_simulation()``.
    frames : dict of (:class:`str`, :class:`tkinter.Frame`)
        A dictionary mapping frame names (e.g., "Simulation", "Controller") to their 
        respective :class:`tkinter.Frame` instances.
    active_frame : :class:`tkinter.Frame`
        The currently displayed content frame.
    """
    def __init__(self):
        """
        Initializes the application window, loads the simulation, and sets up the GUI structure.
        """
        super().__init__()
        self.geometry("1400x800")
        self.title("Satellite Attitude Simulation")
        # Set protocol to handle proper cleanup on window close
        self.protocol('WM_DELETE_WINDOW', self.close_app)

        self.menu_bar = tk.Frame(self, bg="lightgray")
        self.menu_bar.pack(side=tk.TOP, fill="x")

        self.container = tk.Frame(self, bg="white")
        self.container.pack(fill="both", expand=True)

        self.ani_obj: AttitudeAnimation = create_simulation()
        print(self.ani_obj.sat.actuator_system)
        
        # Initialize all major frames/pages
        self.frames: Dict[str, Any] = { 
            "Simulation": SimulationFrame(self.container, self.ani_obj),
            "Controller": ControllerFrame(self.container),
            "Reference": ReferenceFrame(self.container, self.ani_obj.ref),
            "Satellite": SatelliteParamFrame(self.container, self.ani_obj.sat),
            "Actuators": ActuatorSystemParameterFrame(self.container, self.ani_obj.sat.actuator_system),
            "Params" : TimeParamFrame(self.container, TimeParameters())
        }

        self.init_menu()
        
        # Set initial frame to "Simulation"
        self.active_frame: tk.Frame = self.frames["Simulation"]
        self.active_frame.pack(side=tk.TOP, fill="both", expand=True)
        self.frames["Simulation"].reset()

    def init_menu(self):
        """
        Initializes the navigation menu bar with buttons for switching between application frames.
        """
        tk.Button(self.menu_bar, 
                  text="Simulation",
                  command=lambda: self.show_frame("Simulation")).pack(side=tk.LEFT)
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
        """
        Handles the application shutdown protocol.

        Closes all open Matplotlib figures and then destroys the main Tkinter window.
        """
        # Ensure all Matplotlib figures are closed before destroying the GUI
        plt.close("all")
        self.destroy()

    def show_frame(self, page_name: str):
        """
        Switches the currently displayed frame in the main container.

        The currently active frame is hidden, the new frame is packed, and 
        :meth:`update_params` is called to synchronize simulation state.

        Parameters
        ----------
        page_name : str
            The key of the frame to display, corresponding to a key in :attr:`frames`.
        """
        self.active_frame.pack_forget()
        self.active_frame = self.frames[page_name]
        self.active_frame.pack(side=tk.TOP, fill="both", expand=True)
        self.update_params()

    def update_params(self):
        """
        Reads parameters from the various GUI frames and updates the main 
        simulation object (:attr:`ani_obj`).

        This method synchronizes the state of the Controller, Time Parameters, 
        Reference Frame, Satellite properties, and Actuator System between 
        the editable GUI elements and the core simulation logic.
        """
        # Update components of the core simulation object
        self.ani_obj.controller = self.frames["Controller"].get_controller()
        self.ani_obj.time = self.frames["Params"].get_obj()
        self.ani_obj.ref = self.frames["Reference"].get_obj()

        # Rebuild the satellite object and assign its actuator system
        self.ani_obj.sat = self.frames["Satellite"].get_obj()
        self.ani_obj.sat.actuator_system = self.frames["Actuators"].get_obj()

        print("-------------------- Actuator system --------------------")
        print(self.ani_obj.sat.actuator_system)
        print("-------------------------------------------------------------------")

        # Update the actuator system for the display frame for the actuator system and redraw it
        self.frames["Simulation"].act_sys_frame.actuator_system = self.ani_obj.sat.actuator_system
        self.frames["Simulation"].act_sys_frame.reset_display()



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
