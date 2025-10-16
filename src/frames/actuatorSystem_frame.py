from copy import deepcopy
import tkinter as tk
from tkinter import StringVar, ttk, filedialog

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from collections import defaultdict
from typing import Dict, Optional, List, Tuple

from Actuators.actuator import Actuator, PLOTTING_PARAMETES, ActuatorAnimation
from Actuators.actuatorSystem import ActuatorSystem
from animation import BaseAnimation
from frames.base_frame import BaseParamFrame
from frames.actuator_frames import create_actuator_frame, ACTUATOR_FRAME_MAP
from misc import test_page


class ActuatorSystemFrame(tk.Frame):
    """
    A Tkinter Frame designed to display the current state data and associated plots
    for an ActuatorSystem instance.

    This frame is intended for real-time monitoring, showing the last known value
    for key parameters of Reaction Wheels and Magnetorquers.
    """
    def __init__(self, 
                 parent,
                 actuator_system: Optional[ActuatorSystem]=None):
        """
        Initializes the ActuatorSystemFrame.

        Parameters
        ----------
        parent : tk.Widget
            The parent tkinter widget.
        actuator_system : Optional[ActuatorSystem], optional
            The ActuatorSystem instance to display data for. If None, a new
            ActuatorSystem is created. Defaults to None.
        """
        super().__init__(parent)
        self.actuator_system = actuator_system if actuator_system is not None else ActuatorSystem()
        
        # Initialize StringVars for Reaction Wheel (RW) data display
        self.vars_rw: List[Dict[str, StringVar]]= [
            {key: tk.StringVar(value=act.data[key][-1]) for key in act.data.keys() if key != "time"} 
            for act in self.actuator_system.reaction_wheels
        ]
        
        # Initialize StringVars for Magnetorquer (MQT) data display
        self.vars_mqt: List[Dict[str, StringVar]]= [
            {key: tk.StringVar(value=act.data[key][-1]) for key in act.data.keys() if key != "time"} 
            for act in self.actuator_system.magnetorquers
        ]

        self.draw_frame()

    def draw_frame(self):
        """
        Sets up the main layout of the frame, partitioning space for
        actuator values and future plots.
        """
        value_frame = tk.Frame(self)
        plot_frame = tk.Frame(self)

        value_frame.pack(side="left", fill="x", padx=1, pady=1)
        plot_frame.pack(side="left", fill="x", padx=1, pady=1)

        self.draw_actuators(value_frame)

        # WARN: draw_plots functionality does not exist anymore, but might make a comback 
        # at some point
        # self.draw_plots(value_frame)

    def draw_plots(self, frame: tk.Frame):
        """
        Draws the plotting area, setting up a Matplotlib canvas for each
        actuator animation and an OptionMenu to select the data key to plot.

        Parameters
        ----------
        frame : tk.Frame
            The parent frame where plots should be placed.
        """
        colors = ["red", "green", "blue", "magenta", "cyan"]
        for i, anim in enumerate(self.actuator_system.animations):
            anim.color = colors[i % len(colors)]

            row = tk.Frame(frame)
            row.grid(row=i, column=0, sticky="nsew", padx=1, pady=1)

            # Give each row equal "weight" so they share space
            self.rowconfigure(i, weight=1)
            self.columnconfigure(0, weight=1)

            left = tk.Frame(row)
            left.pack(side="left", fill="y", padx=1, pady=1)
            left.pack(side="top", fill="both", padx=1, pady=1, expand=True)

            var = StringVar()
            menu = self.create_actuator_menu(left, anim.actuator, var, lambda var: self.swap_plot(var, i))
            menu.pack(side="top", fill="x", padx=1, pady=1)

            right = tk.Frame(row)
            right.pack(side="left", fill="both", expand=True, padx=1, pady=1)

            canvas = FigureCanvasTkAgg(anim.fig, master=right)
            canvas_widget = canvas.get_tk_widget()
            canvas_widget.pack(side="top", fill="both", expand=True)


    def draw_actuators(self, frame: tk.Frame):
        """
        Draws the individual actuator data displays, showing the name,
        scaled axis vector, and the last recorded value for each data key.

        Parameters
        ----------
        frame : tk.Frame
            The parent frame where actuator information should be placed.
        """

        for i, act in enumerate(self.actuator_system.reaction_wheels):
            rw_frame = tk.Frame(frame)
            rw_frame.grid(row=i+1, column=0, padx=5, pady=2)

            # NOTE: Divide axis by the smallest factor that is not 0 to get a nicer displayed axis
            ax = 1 / np.min([abs(x) for x in act.axis if x != 0.0]) * act.axis
            info_label = tk.Label(rw_frame,
                                  bg="cyan",
                                  text=f"{i+1}: {act.name} -> ({', '.join([f'{x:.2f}' for x in ax])} )")
            info_label.pack(side="top", fill="x")

            # Create a frame to hold the display values
            values_frame = tk.Frame(rw_frame)
            values_frame.pack(side="top", fill="x")
            self.create_values(values_frame, act, self.vars_rw[i])

            div = tk.Frame(rw_frame, bg="black")
            div.pack(side="top", fill="x", pady=2)

        for i, mqt in enumerate(self.actuator_system.magnetorquers):
            mqt_frame = tk.Frame(frame)
            mqt_frame.grid(row=i+1, column=1, sticky="n", padx=5, pady=2)

            # NOTE: Divide axis by the smallest factor that is not 0 to get a nicer displayed axis
            ax = 1 / np.min([abs(x) for x in mqt.axis if x != 0.0]) * mqt.axis
            info_label = tk.Label(mqt_frame,
                                  bg="magenta",
                                  text=f"{i+1}: {mqt.name} -> ({', '.join([f'{x:.2f}' for x in ax])} )")
            info_label.pack(side="top", fill="x")

            # Create a frame to hold the display values
            values_frame = tk.Frame(mqt_frame)
            values_frame.pack(side="top", fill="x")
            self.create_values(values_frame, mqt, self.vars_mqt[i])

            div = tk.Frame(mqt_frame, bg="black")
            div.pack(side="top", fill="x", pady=2)


    def update_values(self):
        """
        Updates the values displayed in the Tkinter labels (via self.vars_rw and self.vars_mqt)
        to reflect the latest (last) data points stored in the actuators' internal data structure.
        """
        # NOTE: Yank implementation but its fine with just 2 types of actuators
        for i, act in enumerate(self.actuator_system.reaction_wheels):
            for key in act.data.keys():
                if key == "time":
                    continue
                value = act.data[key][-1]
                txt = "{:.7f}".format(value)
                self.vars_rw[i][key].set(txt)

        for i, act in enumerate(self.actuator_system.magnetorquers):
            for key in act.data.keys():
                if key == "time":
                    continue
                value = act.data[key][-1]
                txt = "{:.7f}".format(value)
                self.vars_mqt[i][key].set(txt)

    @staticmethod
    def create_actuator_menu(frame: tk.Frame,
                             actuator: Actuator,
                             var: tk.StringVar,
                             cb) -> tk.OptionMenu:
        """
        Creates a Tkinter OptionMenu that allows selection of which data key (e.g., 'torque', 'speed')
        to plot for a given actuator.

        Parameters
        ----------
        frame : tk.Frame
            The parent frame for the menu.
        actuator : Actuator
            The actuator whose data keys will populate the menu.
        var : tk.StringVar
            The StringVar to hold the selected data key.
        cb : Callable
            The callback function executed when a new menu option is selected.

        Returns
        -------
        tk.OptionMenu
            The created OptionMenu widget.
        """
        # keys = [key for key in actuator.data.keys()]
        keys = [key for key in actuator.data.keys() if key != "time"]
        menu = tk.OptionMenu(
            frame, 
            var,
            keys[0],
            *keys[1:],
            command=cb)
        return menu

    @staticmethod
    def create_ref_toggle(frame: tk.Frame, var: tk.IntVar) -> tk.Checkbutton:
        """
        Creates a simple Checkbutton for toggling a reference state.

        Parameters
        ----------
        frame : tk.Frame
            The parent frame for the checkbutton.
        var : tk.IntVar
            The IntVar linked to the checkbutton state.

        Returns
        -------
        tk.Checkbutton
            The created Checkbutton widget.
        """
        toggle = tk.Checkbutton(frame, text="ref", variable=var)
        return toggle

    @staticmethod
    def create_values(frame: tk.Frame,
                      act: Actuator,
                      var: Optional[Dict[str, tk.StringVar]]=None):
        """
        Creates a grid of key-value pair labels for displaying actuator data.
        The layout uses a simple two-column grid (Key | Value).

        Parameters
        ----------
        frame : tk.Frame
            The parent frame (must support grid geometry manager).
        act : Actuator
            The actuator whose data keys and initial values are displayed.
        var : Optional[Dict[str, tk.StringVar]], optional
            A dictionary mapping data keys to tk.StringVars for dynamic updates.
            If provided, the labels use these StringVars; otherwise, static StringVars are created.
            Defaults to None.
        """
        row_num = 0
        bg_color = ["gray", "white"]
        # bg_color = ["yellow", "white"]
        for key in act.data.keys():
            if key == "time":
                continue

            # First Label (Key)
            # Use sticky="w" to left-align within the cell
            tk.Label(frame,
                     text=key,
                     background=bg_color[row_num % 2]).grid(row=row_num,
                                                             column=0,
                                                             sticky="nsew")

            # Second Label (Value)
            # Use a StringVar to update the value
            var_txt = "{:.2f}".format(act.data[key][-1])
            if var is None:
                display_var = tk.StringVar(value=var_txt)
                tk.Label(frame, textvariable=display_var, background=bg_color[row_num % 2]).grid(row=row_num, column=1, sticky="nsew")

            else:
                tk.Label(frame,
                         textvariable=var[key],
                         background=bg_color[row_num % 2]).grid(row=row_num, column=1, sticky="nsew")

            # Add padding to the columns
            frame.columnconfigure(0, weight=1)
            frame.columnconfigure(1, weight=1)

            row_num += 1

    def swap_plot(self, var: StringVar, actuator_idx: int):
        """
        Updates the data key used for plotting a specific actuator's animation.

        Parameters
        ----------
        var : StringVar
            The StringVar containing the newly selected data key (e.g., 'torque').
        actuator_idx : int
            The index of the actuator/animation in the system's animation list.
        """
        # self.actuator_system.animations[actuator_idx].key = var.get()
        # self.actuator_system.animations[actuator_idx].swap_data(var.get())
        self.actuator_system.animations[actuator_idx].swap_data(var)


class ActuatorSystemParameterFrame(tk.Frame):
    """
    A Tkinter Frame for managing and configuring the list of actuators within an
    ActuatorSystem.

    It allows users to add new actuators, delete existing ones, and open pop-up
    windows to edit individual actuator parameters.
    """
    def __init__(self,
                 parent,
                 sys:Optional[ActuatorSystem]=None):
        """
        Initializes the ActuatorSystemParameterFrame.

        Parameters
        ----------
        parent : tk.Widget
            The parent tkinter widget.
        sys : Optional[ActuatorSystem], optional
            The ActuatorSystem instance to configure. If None, a new one is created.
            A deep copy is made to allow for a reset operation. Defaults to None.
        """

        self.parent = parent
        super().__init__(parent)
        self.sys: ActuatorSystem = ActuatorSystem() if sys is None else sys
        self.original: ActuatorSystem = deepcopy(self.sys)
        self.draw_frame()

    def draw_frame(self):
        """
        Draws the main layout of the parameter frame, including the reset button,
        list of current actuators with their delete/options buttons, and the
        UI for adding a new actuator.
        """
        tk.Button(self,
                  text="Reset System",
                  command=lambda: self.reset()).pack(side="top",
                                                     fill="x")
        self.frames = []
        for i, act in enumerate(self.sys.actuators):
            row = tk.Frame(self)
            row.pack(side="top", fill="x")
            tk.Button(row,
                      text="Delete",
                      command=lambda n=i: self.remove_actuator(n)).pack(side="left",
                                                                       fill="x")
            tk.Button(row,
                      text="Options",
                      command=lambda n=i: self.expand(n)).pack(side="left",
                                                               fill="x")

            # NOTE: Divide axis by the smallest factor that is not 0 to get a nicer displayed axis
            ax = 1 / np.min([abs(x) for x in act.axis if x != 0.0]) * act.axis
            s: str = f"Actuator {i+1}: {self.sys.actuators[i].name} | "
            s += f"{i+1}: {ax} -> ({', '.join([f'{x:.2f}' for x in ax])} )"
            tk.Label(row, text=s).pack(side="left", fill="x")
        self.draw_add_actuator()

    def draw_add_actuator(self):
        """
        Draws the UI elements for adding a new actuator, consisting of an
        OptionMenu to select the type and a button to initiate the add/configure process.
        """
        row = tk.Frame(self)
        row.pack(side="top", fill="x")
        self.new_actuator_var = tk.StringVar(value=next(iter(ACTUATOR_FRAME_MAP)).name)
        tk.OptionMenu(row,
                      self.new_actuator_var,
                      *[x.name for x in ACTUATOR_FRAME_MAP.keys()]).pack(side="left",
                                                                         anchor="w")
        tk.Button(row, text="Add Actuator",
                  command=lambda: self.add_actuator()).pack(side="left", fill="x")

    def expand(self, n: int):
        """
        Opens a modal Toplevel window (`popup`) containing a parameter frame
        specific to the actuator at index `n`, allowing the user to modify
        its configuration.

        Parameters
        ----------
        n : int
            The index of the actuator in `self.sys.actuators` to be edited.
        """
        popup = tk.Toplevel(self.parent)
        popup.title("Actuator Parameter Options")
        popup.transient(self.parent)
        popup.grab_set() 

        actuator_instance = self.sys.actuators[n]
        actuator_frame = create_actuator_frame(popup, actuator_instance)

        actuator_frame.pack(side="top", fill="both", expand=True, padx=10, pady=10)
        tk.Button(popup,
                  text="Close",
                  command=lambda: self.update(n, actuator_frame, popup)).pack(pady=10)
        self.parent.wait_window(popup) 

    def remove_actuator(self, n: int):
        """
        Removes the actuator at index `n` from the ActuatorSystem and redraws
        the main frame to update the list.

        Parameters
        ----------
        n : int
            The index of the actuator to remove.
        """
        del self.sys.actuators[n]
        self.redraw()

    def add_actuator(self, *args):
        """
        Instantiates a new actuator based on the type selected in the OptionMenu,
        opens a modal Toplevel window for configuration, and adds the actuator
        to the system upon closing the window.

        Parameters
        ----------
        *args : Any
            Ignored positional arguments (often passed by Tkinter event handlers).
        """
        actuator_instance = None
        for key in ACTUATOR_FRAME_MAP.keys():
            # if key.name == args[0].get():
            if key.name == self.new_actuator_var.get():
                actuator_class = key
                actuator_instance = actuator_class()
                break
        if actuator_instance == None:
            return

        popup = tk.Toplevel(self.parent)
        popup.title("Actuator Parameter Options")
        popup.transient(self.parent)
        popup.grab_set() 
        actuator_frame = create_actuator_frame(popup, actuator_instance)
        actuator_frame.pack(side="top", fill="both", expand=True, padx=10, pady=10)
        tk.Button(popup,
                  text="Close",
                  command=lambda: self.update(-1, actuator_frame, popup)).pack(pady=10)
        self.parent.wait_window(popup) 

    def update(self, n:int, frame: BaseParamFrame, popup: tk.Toplevel):
        """
        Finalizes the actuator configuration from the parameter frame.

        If n is -1, the actuator is appended (new actuator).
        Otherwise, the actuator at index n is replaced (edited actuator).
        Finally, the popup is closed and the main frame is redrawn.

        Parameters
        ----------
        n : int
            The index of the actuator (-1 for a new actuator).
        frame : BaseParamFrame
            The parameter frame containing the configured actuator object.
        popup : tk.Toplevel
            The Toplevel window to be destroyed.
        """
        act = frame.get_obj()

        if n != -1:
            self.sys.actuators[n] = act
        else: 
            self.sys.actuators.append(act)

        popup.destroy()
        self.redraw()

    def redraw(self):
        """
        Clears all existing widgets from the frame and calls `draw_frame()`
        to rebuild the UI with the current list of actuators.
        """
        for widget in self.winfo_children():
            widget.destroy()
        self.draw_frame()
        # print(self.sys)

    def reset(self):
        """
        Resets the ActuatorSystem instance to its state at initialization
        using the stored deep copy (`self.original`) and redraws the frame.
        """
        self.sys = deepcopy(self.original)
        self.redraw()


    def get_obj(self) -> ActuatorSystem:
        """
        Returns the currently configured ActuatorSystem instance.

        Returns
        -------
        ActuatorSystem
            The configured ActuatorSystem.
        """
        return self.sys


if __name__ == '__main__':
    root = tk.Tk()
    test_page(root, ActuatorSystemFrame(root))

    root = tk.Tk()
    test_page(root, ActuatorSystemParameterFrame(root))
