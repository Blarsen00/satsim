import tkinter as tk
from typing import Dict, Optional, Any

from controller import Controller, PDController, SMCController
from misc import test_page

from frames.base_frame import BaseParamFrame

class PDFrame(BaseParamFrame):
    """
    A Tkinter frame for configuring parameters of a Proportional-Derivative (PD) Controller.
    
    It inherits from BaseParamFrame and is designed to display and allow editing
    of the PD controller's specific parameters.
    """
    def __init__(self, parent, controller: Optional[Controller]=None):
        """
        Initializes the PDFrame.

        Args:
            parent: The parent widget (Tkinter container).
            controller (Optional[Controller]): An existing Controller object (expected to be PDController)
                                               to configure. If None, a new PDController is created.
        """
        self.controller = PDController() if controller is None else controller
        param = {key: getattr(self.controller, key) for key in self.controller._params}
        super().__init__(parent, param)

    def draw_frame(self):
        """
        Draws the specific widgets for the PD controller parameters (p and d gains).
        
        It includes a header row for x, y, z labels and input fields for 'p' (field) and 'd' (list field).
        """
        row1 = tk.Frame(self.canvas)
        row1.pack(side=tk.TOP, fill="x")
        tk.Label(row1, text="",
                 width=self.base_width).pack(side=tk.LEFT,
                                             padx=self.base_padding_x,
                                             pady=self.base_padding_y)
        labels = ["x", "y", "z"]
        for label in labels:
            tk.Label(row1, text=label,
                     width=self.base_entry_width).pack(side=tk.LEFT,
                                                       padx=self.base_padding_x,
                                                       pady=self.base_padding_y)

        self.add_field("p: ", self.vars["p"])
        self.add_list_field("d: ", self.vars["d"])

    def reset(self):
        """
        Resets the controller parameters to the default values of a new PDController
        and updates the frame's widgets.

        Returns:
            Any: The result of the parent class's reset method.
        """
        self.controller = PDController()
        self.param = {key: getattr(self.controller, key) for key in self.controller._params}
        return super().reset()
    
    def apply(self):
        """
        Applies the parameters entered in the GUI fields to the internal PDController object.
        """
        super().apply()
        for key in self.param.keys():
            setattr(self.controller, key, self.param[key])
        print(self.controller)

    def get_obj(self) -> Any:
        """
        Returns the configured PDController object.

        Returns:
            Any: The internal PDController instance.
        """
        return self.controller


class SMCFrame(BaseParamFrame):
    """
    A Tkinter frame for configuring parameters of a Sliding Mode Controller (SMC).

    It inherits from BaseParamFrame and is designed to display and allow editing
    of the SMC controller's specific parameters.
    """
    def __init__(self, parent, controller: Optional[Controller]=None):
        """
        Initializes the SMCFrame.

        Args:
            parent: The parent widget (Tkinter container).
            controller (Optional[Controller]): An existing Controller object (expected to be SMCController)
                                               to configure. If None, a new SMCController is created.
        """
        self.controller = SMCController() if controller is None else controller
        param = {key: getattr(self.controller, key) for key in self.controller._params}
        super().__init__(parent, param)

    def draw_frame(self):
        """
        Draws the specific widgets for the SMC controller parameters (k, e, and G).
        
        Includes input fields for 'k' (field), 'e' (field), and 'G' (array field).
        """
        self.add_field("k: ", self.vars["k"])
        self.add_field("e", self.vars["e"])
        self.add_array_field("G: ", self.vars["G"])

    def reset(self):
        """
        Resets the controller parameters to the default values of a new SMCController
        and updates the frame's widgets.

        Returns:
            Any: The result of the parent class's reset method.
        """
        self.controller = SMCController()
        self.param = {key: getattr(self.controller, key) for key in self.controller._params}
        print(self.controller)
        return super().reset()

    def apply(self):
        """
        Applies the parameters entered in the GUI fields to the internal SMCController object.
        """
        super().apply()
        for key in self.param.keys():
            setattr(self.controller, key, self.param[key])
        print(self.controller)

    def get_obj(self) -> Any:
        """
        Returns the configured SMCController object.

        Returns:
            Any: The internal SMCController instance.
        """
        return self.controller


class ControllerFrame(tk.Frame):
    """ 
    A Tkinter frame that allows the user to select a controller type
    and configure its parameters using the appropriate parameter frame. 
    
    New controller types and their corresponding parameter frames must be 
    added to the `self.frames` dictionary here.
    """
    def __init__(self, parent):
        """
        Initializes the ControllerFrame, setting up the available parameter frames
        and drawing the selection interface.

        Args:
            parent: The parent widget (Tkinter container).
        """
        super().__init__(parent)
        self.frames: Dict[str, tk.Frame] = {
            "pd-controller": PDFrame(self),
            "sliding mode controller": SMCFrame(self)
        }
        self.draw()

        self.active_frame: tk.Frame = self.frames["pd-controller"]
        self.active_frame.pack(fill="both", expand=True)

    def draw(self):
        """
        Draws the controller selection mechanism, which is a Tkinter OptionMenu.
        """
        top_frame = tk.Frame(self)
        self.controller_type = tk.StringVar(value="pd-controller")
        top_frame.pack(side="top", fill="x")
        controller_menu = tk.OptionMenu(top_frame, 
                                        self.controller_type,
                                        "pd-controller",
                                        "sliding mode controller",
                                        command=self.update_active_controller)
        tk.Label(top_frame, text="Controller type: ").pack(side=tk.LEFT)
        controller_menu.pack(side=tk.LEFT, pady=5, fill="x")

    def update_active_controller(self, var: tk.StringVar):
        """
        Switches the visible parameter frame based on the controller selected in the OptionMenu.

        Args:
            var (tk.StringVar): The name of the selected controller type (key in self.frames).
        """
        self.active_frame.pack_forget()
        self.active_frame = self.frames[var]
        self.active_frame.pack(side=tk.TOP, fill="x")

    def get_controller(self):
        """
        Retrieves the currently configured controller object from the active parameter frame.

        Returns:
            Controller: The configured controller object (e.g., PDController or SMCController).
        """
        # WARN: Assumes that the frame is a sibclass of BaseParamFrame
        return self.active_frame.get_obj()


if __name__ == '__main__':

    root = tk.Tk()
    test_page(root, ControllerFrame(root))
