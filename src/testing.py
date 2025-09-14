import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from misc import test_page

from attitude import AttitudeSimulation


class test(tk.Frame):
    def __init__(self, parent, obj=AttitudeSimulation()):
        super().__init__(parent)
        # self.anim_obj = AttitudeSimulation()
        self.anim_obj = obj

        self.canvas = FigureCanvasTkAgg(self.anim_obj.fig, master=self)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.var = tk.IntVar(value=1)
        self.box = tk.Checkbutton(self,
                                  text="var",
                                  variable=self.var,
                                  command=lambda: print(f"Checkbox: { self.var.get() }"))
        self.box.pack(side=tk.TOP, fill="both", expand=True)

        self.start_btn = tk.Button(self, text="Start", command=self.anim_obj.animation.resume)
        self.start_btn.pack(side=tk.LEFT)

    def foo(self):
        print(self.var.get())



if __name__ == "__main__":
    root = tk.Tk()
    test_page(root, test(root))
