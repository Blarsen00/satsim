from dataclasses import asdict
import tkinter as tk
from typing import Optional, Any
from attitude import AttitudeAnimation
from misc import test_page

from frames.base_frame import BaseParamFrame

from satellite import Satellite
from simulation import PhysicalState


class SatelliteParamFrame(BaseParamFrame):
    """
    A Tkinter frame for configuring the initial state and properties of a Satellite object.
    
    It inherits from BaseParamFrame and allows editing of the satellite's
    initial attitude, angular rate (PhysicalState), and its inertia matrix (J).
    """
    def __init__(self, parent, sat:Optional[Satellite]=None):
        """
        Initializes the SatelliteParamFrame.

        Args:
            parent: The parent widget (Tkinter container).
            sat (Optional[Satellite]): An existing Satellite object to configure. 
                                       If None, a new Satellite is created.
        """
        self.sat: Satellite = Satellite() if sat is None else sat

        self.param = asdict(self.sat.state)
        # Add non-PhysicalState parameter
        self.param["J"] = self.sat.J
        super().__init__(parent, self.param)

    def draw_frame(self):
        """
        Draws the specific widgets for the satellite parameters.
        
        Includes input fields for Attitude (Quaternion 'rot'), Angular rate ('w'),
        and the Inertia Matrix ('J').
        """
        row = tk.Frame(self.canvas)
        row.pack(side="top", fill="x")
        tk.Label(row, text="State").pack(side="top",
                                         fill="x",
                                         padx=self.base_padding_x,
                                         pady=self.base_padding_y)
        self.add_divider(frame=self.canvas)
        self.add_list_field("Attitude (q): ", self.vars["rot"])
        self.add_list_field("Angular rate (rad/s): ", self.vars["w"])
        self.add_divider(frame=self.canvas)
        self.add_array_field("Inertia Matrix (J): ", self.vars["J"])

    def reset(self):
        """
        Resets the satellite object to a new default Satellite instance 
        and updates the frame's widgets with its default parameters.
        """
        self.sat = Satellite()
        self.param = asdict(self.sat.state)
        self.param["J"] = self.sat.J
        super().reset()

    def apply(self):
        """
        Applies the parameters entered in the GUI fields to the internal 
        Satellite object, updating both the inertia matrix and the PhysicalState.
        """
        super().apply()
        self.sat.J = self.param["J"]
        self.sat.state = PhysicalState(**{key: value for key, value 
                                         in self.param.items() if key != "J"})
        print(self.sat)

    def get_obj(self) -> Any:
        """
        Returns the configured Satellite object.

        Returns:
            Any: The internal Satellite instance.
        """
        return self.sat


class SatelliteFrame(tk.Frame):
    """
    A Tkinter frame designed to display the current attitude and angular rate 
    of both the simulated Satellite and its Reference object.

    It typically works alongside an AttitudeAnimation object to visualize the
    current state values.
    """
    def __init__(self, parent, anim:Optional[AttitudeAnimation]=None):
        """
        Initializes the SatelliteFrame.

        Args:
            parent: The parent widget (Tkinter container).
            anim (Optional[AttitudeAnimation]): The animation object containing 
                                                the current Satellite and Reference states.
                                                If None, a new AttitudeAnimation is created.
        """
        super().__init__(parent)

        self.anim: AttitudeAnimation = AttitudeAnimation() if anim is None else anim
        
        # Initialize Tkinter StringVars for satellite state display
        self.sat_vars = {
            "rot": [tk.StringVar(value=str(x)) for x in self.anim.sat.state.rot.as_quat()],
            "w": [tk.StringVar(value=str(x)) for x in self.anim.sat.state.w],
        }
        # Initialize Tkinter StringVars for reference state display
        self.ref_vars = {
            "rot": [tk.StringVar(value=str(x)) for x in self.anim.ref.state.rot.as_quat()],
            "w": [tk.StringVar(value=str(x)) for x in self.anim.ref.state.w],
        }
        self.draw_frame()


    def draw_frame(self, frame: Optional[tk.Frame]=None):
        """
        Draws the frame containing labels and widgets for displaying 
        Attitude (Quaternion) and Angular Rate for both Satellite and Reference.

        Args:
            frame (Optional[tk.Frame]): The frame to draw the widgets onto. 
                                        Defaults to self (the SatelliteFrame).
        """
        frame = self if frame is None else frame
        width = 12
        bg_ref = "green"
        bg_sat = "blue"

        # Attitude
        tk.Label(frame, text="Attitude").grid(row=0, column=0, columnspan=5)
        tk.Label(frame, text="Satellite: ", anchor="w").grid(row=1, column=0)
        for i in range(4):
            tk.Label(frame,
                     textvariable=self.sat_vars["rot"][i],
                     width=width,
                     bg=bg_sat).grid(row=1, column=i+1)

        tk.Label(frame, text="Reference: ", anchor="w").grid(row=2, column=0)
        for i in range(4):
            tk.Label(frame,
                     textvariable=self.ref_vars["rot"][i],
                     width=width,
                     bg=bg_ref).grid(row=2, column=i+1)

        # Added vertical spacing
        tk.Label(frame, text=" ").grid(row=3, column=0)

        # Angular rate
        tk.Label(frame,
                 text="Angular rate (rad/s)",
                 anchor="w").grid(row=4,
                                  column=0,
                                  columnspan=5)

        tk.Label(frame, text="Satellite: ", anchor="w").grid(row=5, column=0)
        for i in range(3):
            tk.Label(frame,
                     textvariable=self.sat_vars["w"][i],
                     width=width,
                     bg=bg_sat).grid(row=5, column=i+1)

        tk.Label(frame, text="Reference: ", anchor="w").grid(row=6, column=0)
        for i in range(3):
            tk.Label(frame,
                     textvariable=self.ref_vars["w"][i],
                     width=width,
                     bg=bg_ref).grid(row=6, column=i+1)

    def update_values(self):
        """ 
        Updates the Tkinter StringVars with the latest attitude and angular rate 
        values from the internal AttitudeAnimation object's Satellite and Reference.
        Values are formatted to three decimal places.
        """
        for i in range(4):
            sat_q_txt = "{:.3f}".format(self.anim.sat.state.rot.as_quat()[i])
            ref_q_txt = "{:.3f}".format(self.anim.ref.state.rot.as_quat()[i])
            self.sat_vars["rot"][i].set(sat_q_txt)
            self.ref_vars["rot"][i].set(ref_q_txt)
            if i == 3:
                continue
            sat_w_txt = "{:.3f}".format(self.anim.sat.state.w[i])
            ref_w_txt = "{:.3f}".format(self.anim.ref.state.w[i])
            self.sat_vars["w"][i].set(sat_w_txt)
            self.ref_vars["w"][i].set(ref_w_txt)


if __name__ == '__main__':
    # root = tk.Tk()
    # test_page(root, SatelliteParamFrame(root))

    root = tk.Tk()
    test_page(root, SatelliteFrame(root))
