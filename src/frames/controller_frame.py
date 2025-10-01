import tkinter as tk
from typing import Dict, Optional, Any

from controller import Controller, PDController, SMCController
from misc import test_page

from frames.base_frame import BaseParamFrame

class PDFrame(BaseParamFrame):
    def __init__(self, parent, controller: Optional[Controller]=None):
        self.controller = PDController() if controller is None else controller
        param = {key: getattr(self.controller, key) for key in self.controller._params}
        super().__init__(parent, param)

    def draw_frame(self):
        row1 = tk.Frame(self.canvas)
        row1.pack(side=tk.TOP, fill="x")
        tk.Label(row1, text="",
                 width=self.base_width).pack(side=tk.LEFT,
                                             padx=self.base_padding_x,
                                             pady=self.base_padding_y)
        labels = ["x", "y", "z"]
        for label in  labels:
            tk.Label(row1, text=label,
                     width=self.base_entry_width).pack(side=tk.LEFT,
                                                 padx=self.base_padding_x,
                                                 pady=self.base_padding_y)

        self.add_field("p: ", self.vars["p"])
        self.add_list_field("d: ", self.vars["d"])

    def reset(self):
        self.controller = PDController()
        self.param = {key: getattr(self.controller, key) for key in self.controller._params}
        return super().reset()
    
    def apply(self):
        super().apply()
        for key in self.param.keys():
            setattr(self.controller, key, self.param[key])
        print(self.controller)

    def get_obj(self) -> Any:
        return self.controller


class SMCFrame(BaseParamFrame):
    def __init__(self, parent, controller: Optional[Controller]=None):
        self.controller = SMCController() if controller is None else controller
        param = {key: getattr(self.controller, key) for key in self.controller._params}
        super().__init__(parent, param)

    def draw_frame(self):
        self.add_field("k: ", self.vars["k"])
        self.add_field("e", self.vars["e"])
        self.add_array_field("G: ", self.vars["G"])

    def reset(self):
        self.controller = SMCController()
        self.param = {key: getattr(self.controller, key) for key in self.controller._params}
        print(self.controller)
        return super().reset()

    def apply(self):
        super().apply()
        for key in self.param.keys():
            setattr(self.controller, key, self.param[key])
        print(self.controller)

    def get_obj(self) -> Any:
        return self.controller


class ControllerFrame(tk.Frame):
    """ Page for choosing the right controller, and settings
        its parameters. Adding new controllers needs to be done
        here. 
    """
    def __init__(self, parent):
        super().__init__(parent)
        self.frames: Dict[str, tk.Frame] = {
            "pd-controller": PDFrame(self),
            "sliding mode controller": SMCFrame(self)
        }
        self.draw()

        self.active_frame: tk.Frame = self.frames["pd-controller"]
        self.active_frame.pack(fill="both", expand=True)

    def draw(self):
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
        self.active_frame.pack_forget()
        self.active_frame = self.frames[var]
        self.active_frame.pack(side=tk.TOP, fill="x")

    def get_controller(self):
        # WARN: Assumes that the frame is a sibclass of BaseParamFrame
        return self.active_frame.get_obj()


if __name__ == '__main__':
    # root = tk.Tk()
    # test_page(root, PDFrame(root))
    #
    # root = tk.Tk()
    # test_page(root, SMCFrame(root))

    root = tk.Tk()
    test_page(root, ControllerFrame(root))
