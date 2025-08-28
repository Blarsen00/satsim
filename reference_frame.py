import tkinter as tk

from scipy.spatial.transform import Rotation
from base_frame import BaseParamFrame
from simulation import PhysicalState
from misc import test_page


class ReferenceFrame(BaseParamFrame):
    def __init__(self, parent, param=None):
        self.param = PhysicalState() if param is None else param
        super().__init__(parent, self.param)

        self.q_vars = [tk.StringVar(value=str(self.param.rot.as_quat()[i]))
                        for i in range(4)]

        self.draw_frame()


    def reset(self):
        self.param = PhysicalState()
        self.update_values(self.param)


    def draw_frame(self):
        row1 = tk.Frame(self.canvas)
        row1.pack(side=tk.TOP, fill="x")
        tk.Label(row1, text="",
                 width=self.base_width).pack(side=tk.LEFT,
                                             padx=self.base_padding_x,
                                             pady=self.base_padding_y)
        tk.Label(row1, text="X", 
                 width=self.base_entry_width).pack(side=tk.LEFT,
                                                   padx=self.base_padding_x,
                                                   pady=self.base_padding_y)
        tk.Label(row1, text="Y",
                 width=self.base_entry_width).pack(side=tk.LEFT,
                                                   padx=self.base_padding_x,
                                                   pady=self.base_padding_y)
        tk.Label(row1, text="Z", 
                 width=self.base_entry_width).pack(side=tk.LEFT,
                                                   padx=self.base_padding_x,
                                                   pady=self.base_padding_y)
        tk.Label(row1, text="W", 
                 width=self.base_entry_width).pack(side=tk.LEFT,
                                                   padx=self.base_padding_x,
                                                   pady=self.base_padding_y)

        row2 = tk.Frame(self.canvas)
        row2.pack(side=tk.TOP, fill="x")
        tk.Label(row2, text="Start quaternion: ",
                 width=self.base_width).pack(side=tk.LEFT,
                                             padx=self.base_padding_x,
                                             pady=self.base_padding_y)
        tk.Entry(row2,
                 width=self.base_entry_width,
                 textvariable=self.q_vars[0]).pack(side=tk.LEFT,
                                                   padx=self.base_padding_x,
                                                   pady=self.base_padding_y)
        tk.Entry(row2,
                 width=self.base_entry_width,
                 textvariable=self.q_vars[1]).pack(side=tk.LEFT,
                                                   padx=self.base_padding_x,
                                                   pady=self.base_padding_y)
        tk.Entry(row2,
                 width=self.base_entry_width,
                 textvariable=self.q_vars[2]).pack(side=tk.LEFT,
                                                   padx=self.base_padding_x,
                                                   pady=self.base_padding_y)
        tk.Entry(row2, 
                 width=self.base_entry_width,
                 textvariable=self.q_vars[3]).pack(side=tk.LEFT, 
                                                   padx=self.base_padding_x,
                                                   pady=self.base_padding_y)

    def apply(self):
        q = [float(self.q_vars[i].get()) for i in range(4)]
        setattr(self.param, "rot", Rotation.from_quat(q))
        return super().apply()
    

    def update_values(self, param=None):
        super().update_values(param)
        for i in range(4):
            self.q_vars[i].set(str(self.param.rot.as_quat()[i]))



if __name__ == '__main__':
    root = tk.Tk()
    test_page(root, ReferenceFrame(root))


