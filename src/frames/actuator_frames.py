from copy import deepcopy
import tkinter as tk
from typing import Dict, Optional, Type, Union
from Actuators.actuator import Actuator
from Actuators.magnetorquer import Magnetorquer
from frames.base_frame import BaseParamFrame
from Actuators.reactionwheel import ReactionWheel, make_reactionwheel_plot
from misc import test_page

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class ReactionWheelFrame(BaseParamFrame):
    """
    A Tkinter frame used for viewing and modifying the parameters of a
    ReactionWheel object.

    Inherits from BaseParamFrame to handle parameter binding and application logic.
    It also includes a visualization plot specific to the ReactionWheel.
    """
    def __init__(self,
                 parent: tk.Widget,
                 rw: Optional[ReactionWheel]=None):
        """
        Initializes the ReactionWheelFrame.

        Parameters
        ----------
        parent : tk.Widget
            The parent tkinter widget.
        rw : Optional[ReactionWheel], optional
            The ReactionWheel instance to configure. If None, a new default
            ReactionWheel is created. Defaults to None.
        """
        self.rw = ReactionWheel() if rw is None else rw
        self.original = deepcopy(self.rw)

        param = {key: getattr(self.rw, key) for key in self.rw._params}
        super().__init__(parent, param)

    def draw_frame(self):
        """
        Constructs the UI for parameter input and the ReactionWheel visualization plot.
        """
        params = tk.Frame(self.canvas)
        params.pack(side="left",
                    fill="x",
                    pady=self.base_padding_y,
                    padx=self.base_padding_x)

        self.add_divider(frame=params)
        self.add_list_field("Axis (x, y, z): ", self.vars["axis"], frame=params)
        self.add_field("Anuglar rate, w (rad/s)", self.vars["w"], frame=params)
        self.add_divider(frame=params)

        self.add_array_field("J: ", self.vars["J"], frame=params)
        self.add_divider(frame=params)

        self.add_field("Max current I: ", self.vars["max_I"], frame=params)
        self.add_field("Motor constant Km", self.vars["km"], frame=params)
        self.add_divider(frame=params)

        self.add_field("Coloumb damping coefficient (dc): ", self.vars["dc"], frame=params)
        self.add_field("Viscous damping coefficient (dv): ", self.vars["dv"], frame=params)

        # Plotting frame for visualization
        self.plot_frame = tk.Frame(self.canvas)
        self.plot_frame.pack(side="left",
                             fill="x",
                             pady=self.base_padding_y,
                             padx=self.base_padding_x)
        self.rw_plot = make_reactionwheel_plot(self.rw)
        self.rw_canvas = FigureCanvasTkAgg(self.rw_plot, master=self.plot_frame)
        self.rw_canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

    def reset(self):
        """
        Resets all parameters in the frame back to the object's original state.
        """
        self.rw = deepcopy(self.original)
        self.param = {key: getattr(self.rw, key) for key in self.rw._params}
        return super().reset()

    def apply(self):
        """
        Applies the configured parameters from the UI back to the ReactionWheel object,
        and then redraws the frame to update visualizations.
        """
        super().apply()
        for key in self.param.keys():
            setattr(self.rw, key, self.param[key])
        self.redraw()
        print(self.rw)

    def get_obj(self) -> ReactionWheel:
        """
        Returns the configured ReactionWheel object.

        Returns
        -------
        ReactionWheel
            The updated ReactionWheel instance.
        """
        return self.rw


class MagnetorquerFrame(BaseParamFrame):
    """
    A Tkinter frame used for viewing and modifying the parameters of a
    Magnetorquer object.

    It includes configuration fields and a display of the calculated maximum torque.
    """
    def __init__(self,
                 parent: tk.Widget,
                 mqt: Optional[Magnetorquer]=None):
        """
        Initializes the MagnetorquerFrame.

        Parameters
        ----------
        parent : tk.Widget
            The parent tkinter widget.
        mqt : Optional[Magnetorquer], optional
            The Magnetorquer instance to configure. If None, a new default
            Magnetorquer is created. Defaults to None.
        """
        self.mqt: Magnetorquer = Magnetorquer() if mqt is None else mqt
        self.original = deepcopy(self.mqt)

        param = {key: getattr(self.mqt, key) for key in self.mqt._params}
        super().__init__(parent, param)

    def draw_frame(self) -> None:
        """
        Constructs the UI for parameter input, including the magnetic dipole
        scalar, max current, and a derived maximum torque display.
        """
        params = tk.Frame(self.canvas)
        params.pack(side="left",
                    fill="x",
                    pady=self.base_padding_y,
                    padx=self.base_padding_x)

        self.add_divider(frame=params)
        self.add_list_field("Axis (x, y, z): ", self.vars["axis"], frame=params)
        self.add_field("Magnetic dipole scalar (k)", self.vars["k"], frame=params)
        self.add_field("Max current I (A)", self.vars["max_I"], frame=params)

        # Calculate and display the maximum theoretical torque
        self.max_torque_var = tk.StringVar(value=str(float(self.vars["k"].get())
                                                     * float(self.vars["max_I"].get())
                                                     * 65e-6))
        
        max_torque_frame = tk.Frame(params)
        max_torque_frame.pack(side="top", fill="x")
        tk.Label(max_torque_frame,
                 text="Max torque (B=65 μT): ").grid(row=0, column=0)
        tk.Label(max_torque_frame, textvariable=self.max_torque_var).grid(row=0, column=1)
        tk.Label(max_torque_frame, text="Nm").grid(row=0, column=2)
        self.add_divider(frame=params)

    def calculate_max_torque(self):
        """
        Calculates and updates the displayed maximum torque value based on the
        current 'k' and 'max_I' parameters, assuming a magnetic field strength
        of 65 micro Tesla (B=65e-6 T).
        """
        try:
            k = float(self.vars["k"].get())
            max_I = float(self.vars["max_I"].get())
            tau_m = k * max_I * 65e-6
            self.max_torque_var.set(f"{tau_m:.3e}")
        except ValueError:
            # Handle cases where input fields are empty or non-numeric
            self.max_torque_var.set("N/A")

    def reset(self) -> None:
        """
        Resets all parameters in the frame back to the object's original state
        and recalculates the max torque display.
        """
        self.mqt = deepcopy(self.original)
        self.param = {key: getattr(self.mqt, key) for key in self.mqt._params}
        self.calculate_max_torque()
        return super().reset()

    def apply(self) -> None:
        """
        Applies the configured parameters from the UI back to the Magnetorquer object,
        and then updates the calculated max torque display.
        """
        super().apply()
        for key in self.param.keys():
            setattr(self.mqt, key, self.param[key])
        self.calculate_max_torque()
        self.redraw()
        print(self.mqt)

    def get_obj(self) -> Magnetorquer:
        """
        Returns the configured Magnetorquer object.

        Returns
        -------
        Magnetorquer
            The updated Magnetorquer instance.
        """
        return self.mqt


def create_actuator_frame(parent: tk.Widget, actuator: Optional[Union[Type[Actuator], Actuator]]) -> tk.Frame:
    """ 
    Factory function that creates and returns the appropriate parameter frame 
    (e.g., ReactionWheelFrame, MagnetorquerFrame) for the given actuator object 
    based on the ACTUATOR_FRAME_MAP.

    Parameters
    ----------
    parent : tk.Widget
        The parent tkinter widget for the new frame.
    actuator : Optional[Union[Type[Actuator], Actuator]]
        An instance of an Actuator subclass (or the class itself) to determine
        which configuration frame to create.

    Returns
    -------
    tk.Frame
        An instance of the specific parameter frame (e.g., ReactionWheelFrame).

    Raises
    ------
    ValueError
        If no parameter frame is registered for the actuator type in ACTUATOR_FRAME_MAP.
    """
    actuator_class = type(actuator)
    FrameClass: Type[BaseParamFrame] = ACTUATOR_FRAME_MAP.get(actuator_class)
    
    if FrameClass is None:
        raise ValueError(
            f"No parameter frame registered for actuator type: {actuator_class.__name__}"
        )
    return FrameClass(parent, actuator)


""" 
Mapping from actuator class to its corresponding parameter configuration frame class.
This dictionary is used by create_actuator_frame to instantiate the correct UI.
"""
ACTUATOR_FRAME_MAP: Dict[Type[Actuator], Type[BaseParamFrame]] = {
    ReactionWheel: ReactionWheelFrame,
    Magnetorquer: MagnetorquerFrame
}

if __name__ == '__main__':
    rw = ReactionWheel(axis=np.array([0.0, 0.0, 1.0]))
    root = tk.Tk()
    test_page(root, ReactionWheelFrame(root))

    mqt = Magnetorquer()
    root = tk.Tk()
    test_page(root, MagnetorquerFrame(root, mqt))

    root = tk.Tk()
    test_page(root, create_actuator_frame(root, mqt))
