import tkinter as tk

from reference import BaseReference
from simulation import PhysicalState
from misc import test_page

from frames.base_frame import BaseParamFrame

from typing import Optional, Any
from dataclasses import asdict

class ReferenceFrame(BaseParamFrame):
    """
    A Tkinter frame for configuring the parameters of a BaseReference object,
    which typically defines a target or reference state (PhysicalState) for a system.

    It inherits from BaseParamFrame and allows the user to edit the reference
    state's properties, such as orientation (Quaternion) and angular velocity.
    """
    def __init__(self, parent, reference: Optional[BaseReference]=None):
        """
        Initializes the ReferenceFrame.

        Args:
            parent: The parent widget (Tkinter container).
            reference (Optional[BaseReference]): An existing BaseReference object 
                                                 to configure. If None, a new BaseReference 
                                                 is created. The parameters are extracted 
                                                 from the `reference.state` dataclass.
        """
        self.reference = BaseReference() if reference is None else reference
        self.param = asdict(self.reference.state)
        super().__init__(parent, self.param)

    def reset(self):
        """
        Resets the reference object to a new default BaseReference instance 
        and updates the frame's widgets with its default parameters.

        Returns:
            Any: The result of the parent class's reset method.
        """
        self.reference = BaseReference()
        self.param = asdict(self.reference.state)
        return super().reset()

    def apply(self):
        """
        Applies the parameters entered in the GUI fields to the internal 
        BaseReference object by creating a new PhysicalState.
        """
        super().apply()
        print(self.param)
        self.reference.state = PhysicalState(**self.param)

    def get_obj(self) -> Any:
        """
        Returns the configured BaseReference object.

        Returns:
            Any: The internal BaseReference instance.
        """
        return self.reference

    def draw_frame(self):
        """
        Draws the specific widgets for configuring the reference state,
        including fields for Quaternion and Angular Velocity.
        """
        row = tk.Frame(self.canvas)
        row.pack(side=tk.TOP, fill="x")

        tk.Label(row, text="",
                 width=self.base_width).pack(side=tk.LEFT,
                                             padx=self.base_padding_x,
                                             pady=self.base_padding_y)
        labels = ["x", "y", "z", "w"]
        for label in labels:
            tk.Label(row, text=label,
                     width=self.base_entry_width).pack(side=tk.LEFT,
                                                       padx=self.base_padding_x,
                                                       pady=self.base_padding_y)

        self.add_list_field("Quaternion: ", self.vars["rot"])
        self.add_list_field("Angular Velocity: ", self.vars["w"])


if __name__ == '__main__':
    root = tk.Tk()
    test_page(root, ReferenceFrame(root))
