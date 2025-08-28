import tkinter as tk
from typing import Dict

from controller import Controller, PDParameters, SMCParameters
from base_frame import BaseParamFrame
from misc import test_page


class PDFrame(BaseParamFrame):
    def __init__(self, parent, param=None):
        self.param = PDParameters() if param is None else param
        super().__init__(parent, self.param)
        self.draw_frame()


    def reset(self):
        self.param = PDParameters()
        self.update_values(self.param)


    def draw_frame(self):
        # Top row
        row1 = tk.Frame(self.canvas)
        row1.pack(side=tk.TOP, fill="x")
        tk.Label(row1, text="",
                 width=self.base_width).pack(side=tk.LEFT,
                                             padx=self.base_padding_x,
                                             pady=self.base_padding_y)
        tk.Label(row1,
                 width=self.base_entry_width,
                 text="X").pack(side=tk.LEFT,
                                padx=self.base_padding_x,
                                pady=self.base_padding_y)
        tk.Label(row1, 
                 width=self.base_entry_width,
                 text="Y").pack(side=tk.LEFT,
                                padx=self.base_padding_x,
                                pady=self.base_padding_y)
        tk.Label(row1,
                 width=self.base_entry_width,
                 text="Z").pack(side=tk.LEFT,
                                padx=self.base_padding_x,
                                pady=self.base_padding_y)

        # Second row
        row2 = tk.Frame(self.canvas)
        row2.pack(side=tk.TOP, fill="x")
        tk.Label(row2, width=self.base_width, text="Proportional:").pack(side=tk.LEFT)
        tk.Entry(row2,
                 width=self.base_entry_width,
                 textvariable=self.param_vars["px"]).pack(side=tk.LEFT,
                                                          padx=self.base_padding_x,
                                                          pady=self.base_padding_y)
        tk.Entry(row2, width=self.base_entry_width,
                 textvariable=self.param_vars["py"]).pack(side=tk.LEFT,
                                                          padx=self.base_padding_x,
                                                          pady=self.base_padding_y)
        tk.Entry(row2, width=self.base_entry_width,
                 textvariable=self.param_vars["pz"]).pack(side=tk.LEFT,
                                                          padx=self.base_padding_x,
                                                          pady=self.base_padding_y)

        # Third row
        row3 = tk.Frame(self.canvas)
        row3.pack(side=tk.TOP, fill="x")
        tk.Label(row3, width=self.base_width, text="Derivative:").pack(side=tk.LEFT)
        tk.Entry(row3, width=self.base_entry_width,
                 textvariable=self.param_vars["dx"]).pack(side=tk.LEFT,
                                                          padx=self.base_padding_x,
                                                          pady=self.base_padding_y)
        tk.Entry(row3, width=self.base_entry_width,
                 textvariable=self.param_vars["dy"]).pack(side=tk.LEFT,
                                                          padx=self.base_padding_x,
                                                          pady=self.base_padding_y)
        tk.Entry(row3, width=self.base_entry_width,
                 textvariable=self.param_vars["dz"]).pack(side=tk.LEFT,
                                                          padx=self.base_padding_x,
                                                          pady=self.base_padding_y)


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
        # Top row
        row1 = tk.Frame(self.canvas)
        row1.pack(side=tk.TOP, fill="x")
        tk.Label(row1,
                 text="k:",
                 width=self.base_width).pack(side=tk.LEFT,
                                             padx=self.base_padding_x,
                                             pady=self.base_padding_y)
        tk.Entry(row1,
                 width=self.base_entry_width,
                 textvariable=self.param_vars["k"]).pack(side=tk.LEFT,
                                                         padx=self.base_padding_x,
                                                         pady=self.base_padding_y)

        # Second row
        row2 = tk.Frame(self.canvas)
        row2.pack(side=tk.TOP, fill="x")
        tk.Label(row2,
                 width=self.base_width,
                 text="e:").pack(side=tk.LEFT,
                                 padx=self.base_padding_x, 
                                 pady=self.base_padding_y)
        tk.Entry(row2, 
                 width=self.base_entry_width,
                 textvariable=self.param_vars["e"]).pack(side=tk.LEFT,
                                                         padx=self.base_padding_x,
                                                         pady=self.base_padding_y)
        tk.Label(self.canvas, text="",
                 width=self.base_width).pack(side=tk.TOP, 
                                             fill="x",
                                             padx=self.base_padding_x,
                                             pady=self.base_padding_y)

        # First G row
        row_G1 = tk.Frame(self.canvas)
        row_G1.pack(side=tk.TOP, fill="x")
        tk.Label(row_G1,
                 width=self.base_width,
                 text="").pack(side=tk.LEFT,
                               padx=self.base_padding_x,
                               pady=self.base_padding_y)
        tk.Entry(row_G1, width=self.base_entry_width,
                 textvariable=self.G_vars[0][0]).pack(side=tk.LEFT,
                                                      padx=self.base_padding_x,
                                                      pady=self.base_padding_y)
        tk.Entry(row_G1, width=self.base_entry_width,
                 textvariable=self.G_vars[0][1]).pack(side=tk.LEFT,
                                                      padx=self.base_padding_x,
                                                      pady=self.base_padding_y)
        tk.Entry(row_G1, width=self.base_entry_width,
                 textvariable=self.G_vars[0][2]).pack(side=tk.LEFT,
                                                      padx=self.base_padding_x,
                                                      pady=self.base_padding_y)

        # Second G row
        row_G2 = tk.Frame(self.canvas)
        row_G2.pack(side=tk.TOP, fill="x")
        tk.Label(row_G2,
                 width=self.base_width,
                 text="G: ").pack(side=tk.LEFT,
                                  padx=self.base_padding_x,
                                  pady=self.base_padding_y)
        tk.Entry(row_G2,
                 width=self.base_entry_width,
                 textvariable=self.G_vars[1][0]).pack(side=tk.LEFT,
                                                      padx=self.base_padding_x,
                                                      pady=self.base_padding_y)
        tk.Entry(row_G2,
                 width=self.base_entry_width,
                 textvariable=self.G_vars[1][1]).pack(side=tk.LEFT,
                                                      padx=self.base_padding_x,
                                                      pady=self.base_padding_y)
        tk.Entry(row_G2,
                 width=self.base_entry_width,
                 textvariable=self.G_vars[1][2]).pack(side=tk.LEFT,
                                                      padx=self.base_padding_x,
                                                      pady=self.base_padding_y)

        # Third G row
        row_G3 = tk.Frame(self.canvas)
        row_G3.pack(side=tk.TOP, fill="x")
        tk.Label(row_G3,
                 width=self.base_width,
                 text="").pack(side=tk.LEFT,
                               padx=self.base_padding_x,
                               pady=self.base_padding_y)
        tk.Entry(row_G3,
                 width=self.base_entry_width,
                 textvariable=self.G_vars[2][0]).pack(side=tk.LEFT,
                                                      padx=self.base_padding_x,
                                                      pady=self.base_padding_y)
        tk.Entry(row_G3, width=self.base_entry_width,
                 textvariable=self.G_vars[2][1]).pack(side=tk.LEFT,
                                                      padx=self.base_padding_x,
                                                      pady=self.base_padding_y)
        tk.Entry(row_G3, width=self.base_entry_width,
                 textvariable=self.G_vars[2][2]).pack(side=tk.LEFT,
                                                      padx=self.base_padding_x,
                                                      pady=self.base_padding_y)

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
