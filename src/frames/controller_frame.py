import tkinter as tk
from typing import Dict

from controller import Controller, PDParameters, SMCParameters
from misc import test_page

from frames.base_frame import BaseParamFrame

class PDFrame(BaseParamFrame):
    def __init__(self, parent, param=None):
        self.param = PDParameters() if param is None else param
        super().__init__(parent, self.param)
        self.p_var = tk.StringVar(value=str(getattr(self.param, "p")))
        # self.p_vars = [tk.StringVar(value=str(getattr(self.param, "p")[i]))
                       # for i in range(len(getattr(self.param, "p")))]
        self.d_vars = [tk.StringVar(value=str(getattr(self.param, "d")[i]))
                       for i in range(len(getattr(self.param, "d")))]
        self.draw_frame()


    def reset(self):
        self.param = PDParameters()
        self.update_values(self.param)


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

        # self.add_list_field("p: ", self.p_vars)
        self.add_field("p: ", self.p_var)
        self.add_list_field("d: ", self.d_vars)


    def apply(self):
        # p = [float(self.p_vars[i].get()) for i in range(len(self.p_vars))]
        p = float(self.p_var.get())
        d = [float(self.d_vars[i].get()) for i in range(len(self.d_vars))]
        setattr(self.param, "p", p)
        setattr(self.param, "d", d)
        return super().apply()


    def update_values(self, param=None):
        for i in range(len(self.d_vars)):
            # self.p_vars[i].set(str(self.param.p[i]))
            self.d_vars[i].set(str(self.param.d[i]))
        return super().update_values(param)


class SMCFrame(BaseParamFrame):
    def __init__(self, parent, param=None):
        self.param = SMCParameters() if param is None else param
        super().__init__(parent, self.param)

        self.G_vars = [[tk.StringVar(value=str(self.param.G[i][j]))
                        for j in range(3)] for i in range(3)]
        self.draw_frame()


    def reset(self):
        self.param = SMCParameters()
        self.update_values(self.param)


    def draw_frame(self):
        self.add_field("k: ", self.param_vars["k"])
        self.add_field("e: ", self.param_vars["e"])
        self.add_array_field("G: ", self.G_vars)


    def apply(self):
        G = [[float(self.G_vars[i][j].get()) for j in range(3)]
            for i in range(3)]
        setattr(self.param, "G", G)
        return super().apply()


    def update_values(self, param=None):
        super().update_values(param)
        for i in range(3):
            for j in range(3):
                self.G_vars[i][j].set(str(self.param.G[i][j]))


class ControllerFrame(tk.Frame):
    """
        Page for choosing the right controller, and settings
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
        # controller = self.controller_type.get()
        self.active_frame.pack_forget()
        self.active_frame = self.frames[var]
        self.active_frame.pack(side=tk.TOP, fill="x")


    def get_controller(self):
        return Controller.get_controller(self.active_frame.param)



if __name__ == '__main__':
    root = tk.Tk()
    test_page(root, PDFrame(root))

    root = tk.Tk()
    test_page(root, SMCFrame(root))

    root = tk.Tk()
    test_page(root, ControllerFrame(root))
