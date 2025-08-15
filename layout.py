import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import numpy as np

from parameters import *

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from mpl_toolkits.mplot3d import Axes3D

class App(tk.Tk):
    def __init__(self):
        super().__init__()

        # PLT keeps the window open if the plot is not closed. See close_app
        self.protocol('WM_DELETE_WINDOW', self.close_app)

        self.title("Satellite Attitude Simulation")
        self.geometry("800x400")

        # Create menu bar (you can style this further)
        self.menu_bar = tk.Frame(self, bg="lightgray")
        self.menu_bar.pack(side="top", fill="x")

        # Create main container for pages
        self.container = tk.Frame(self)
        self.container.pack(fill="both", expand=True)

        # Dictionary to store frames
        self.frames = {}

        for F in (HomePage, SettingsPage, AboutPage, SimulationPage, ControllerPage):
            page_name = F.__name__
            frame = F(parent=self.container)
            self.frames[page_name] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        # Create buttons for menu
        tk.Button(self.menu_bar, text="Home", command=lambda: self.show_frame("HomePage")).pack(side="left")
        tk.Button(self.menu_bar, text="Settings", command=lambda: self.show_frame("SettingsPage")).pack(side="left")
        tk.Button(self.menu_bar, text="About", command=lambda: self.show_frame("AboutPage")).pack(side="left")
        tk.Button(self.menu_bar, text="Simulation", command=lambda: self.show_frame("SimulationPage")).pack(side="left")
        tk.Button(self.menu_bar, text="Controller", command=lambda: self.show_frame("ControllerPage")).pack(side="left")

        # Show default frame
        self.show_frame("HomePage")

    def close_app(self):
        # matplotlib needs to be stopped in addition to the root
        plt.close()
        self.destroy()

    def show_frame(self, page_name):
        frame = self.frames[page_name]
        frame.tkraise()

class HomePage(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        label = tk.Label(self, text="Welcome to the Home Page!")

        # Control frame for some options
        controls_frame = tk.Frame(self)
        controls_frame.pack(side="left", fill="y", padx=10, pady=10)

        self.start_btn = ttk.Button(controls_frame, text="Start", command=self.start_animation)
        self.start_btn.pack(fill="x", pady=5)

        self.stop_btn = ttk.Button(controls_frame, text="Stop", command=self.stop_animation)
        self.stop_btn.pack(fill="x", pady=5)

        self.reference_checkbox = tk.Checkbutton(
                controls_frame,
                text="Reference",
                onvalue=True,
                offvalue=False)
        self.reference_checkbox.pack(fill="x", pady=5)

        self.plot_layout = ttk.Combobox(
                controls_frame,
                values=["3D-Plot",
                        "Torque",
                        "Angular Velocity",
                        "Quaternion",
                        "3D + Torque",
                        "3D + Angular Velocity",
                        "3D + Quaternion",],
                state="readonly",
        )
        self.plot_layout.pack(fill="x", pady=5)
        self.plot_layout.current(0)

        # Frame for the plot
        plot_frame = tk.Frame(self)
        plot_frame.pack(side="right", fill="both", expand=True)

        # Create Matplotlib Figure
        self.fig = plt.figure(figsize=(5, 3), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')  # 3D axes
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.set_zlim(-1.5, 1.5)

        # Initial 3D line
        self.line, = self.ax.plot3D([], [], [], lw=2)

        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.t = 0  # time

        # Canvas in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)

        self.ani = None  # animation handle


    def start_animation(self):
        if self.ani is None:
            self.ani = FuncAnimation(
                self.fig, self.update_plot, interval=50, blit=False
            )
        self.canvas.draw()


    def stop_animation(self):
        if self.ani:
            self.ani.event_source.stop()
            self.ani = None  # Reset animation handle


    def update_plot(self, frame):
        self.t += 0.1
        self.x_data.append(self.t)
        self.y_data.append(np.sin(self.t))
        self.z_data.append(np.cos(self.t))

        # Keep last 100 points
        self.x_data = self.x_data[-100:]
        self.y_data = self.y_data[-100:]
        self.z_data = self.z_data[-100:]

        # Update line data for 3D
        self.line.set_data(self.x_data, self.y_data)
        self.line.set_3d_properties(self.z_data)

        self.ax.set_xlim(self.x_data[0], self.x_data[-1])
        return self.line,

    #     # Create Matplotlib Figure
    #     self.fig = plt.figure(figsize=(5, 3), dpi=100)
    #     self.ax = self.fig.add_subplot(111)
    #     self.ax.set_ylim(-1.5, 1.5)
    #     self.line, = self.ax.plot([], [], lw=2)
    #
    #     self.x_data = []
    #     self.y_data = []
    #     self.t = 0  # time
    #
    #     # Canvas in Tkinter
    #     self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
    #     self.canvas.draw()
    #     self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)
    #
    #     self.ani = None  # animation handle
    #
    # def close_app(self):
    #     pass
    #
    # def start_animation(self):
    #     if self.ani is None:
    #         self.ani = FuncAnimation(
    #             self.fig, self.update_plot, interval=50, 
    #             blit=False
    #         )
    #     self.canvas.draw()
    #
    # def stop_animation(self):
    #     if self.ani:
    #         self.ani.event_source.stop()
    #         self.ani = None  # Reset animation handle
    #
    # def update_plot(self, frame):
    #     self.t += 0.1
    #     self.x_data.append(self.t)
    #     self.y_data.append(np.sin(self.t))
    #
    #     # Keep last 100 points
    #     self.x_data = self.x_data[-100:]
    #     self.y_data = self.y_data[-100:]
    #
    #     self.line.set_data(self.x_data, self.y_data)
    #     self.ax.set_xlim(self.x_data[0], self.x_data[-1])
    #     return self.line,

class SettingsPage(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        label = ttk.Label(self, text="Settings Page")
        label.pack(pady=20)

class ControllerPage(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.pd_params = PDParameters()
        self.smc_params = SMCParameters()

        self.controller_type = tk.StringVar(value="PD")
        ttk.Label(self, text="Select Controller Type:").grid(row=0, column=0, sticky="w", columnspan=1)
        controller_menu = ttk.OptionMenu(self, self.controller_type, "PD", "PD", "SMC", command=self.update_active_controller)
        controller_menu.grid(row=0, column=1, sticky="w")

        # PD Fields
        self.pd_entries_values = [tk.StringVar(value=str(getattr(self.pd_params, f.name))) for f in fields(self.pd_params)]
        self.pd_entries = []
        ttk.Label(self, text="PD Parameters:").grid(row=1, column=0, columnspan=1)
        ttk.Label(self, text="X").grid(row=1, column=1, columnspan=1)
        ttk.Label(self, text="Y").grid(row=1, column=2, columnspan=1)
        ttk.Label(self, text="Z").grid(row=1, column=3, columnspan=1)
        ttk.Label(self, text="P: ").grid(row=2, column=0, sticky="e")
        ttk.Label(self, text="D: ").grid(row=3, column=0, sticky="e")

        for i, var in enumerate(self.pd_entries_values):
            entry = tk.Entry(self, textvariable=var, width=5)
            entry.grid(row=2 + (i // 3), column=1 + (i % 3))
            self.pd_entries.append(entry)

        # SMC Fields
        ttk.Label(self, text="SMC Parameters:").grid(row=5, column=0, columnspan=2, pady=(10, 0), sticky='w')
        ttk.Label(self, text="G: ").grid(row=7, column=0, columnspan=1, sticky='e')
        self.G_entries = []
        self.G_vars = [[tk.StringVar(value=str(self.smc_params.G[i][j])) for j in range(3)] for i in range(3)]

        for i in range(3):
            for j in range(3):
                entry = tk.Entry(self, textvariable=self.G_vars[i][j], width=5)
                entry.grid(row=6 + i, column=j + 1)
                self.G_entries.append(entry)

        ttk.Label(self, text="k:").grid(row=9, column=0, sticky="e")
        self.k_var = tk.StringVar(value=str(self.smc_params.k))
        self.k_entry = tk.Entry(self, textvariable=self.k_var, width=5)
        self.k_entry.grid(row=9, column=1, sticky="e")

        ttk.Label(self, text="e:").grid(row=10, column=0, sticky="e")
        self.e_var = tk.StringVar(value=str(self.smc_params.e))
        self.e_entry = tk.Entry(self, textvariable=self.e_var, width=5)
        self.e_entry.grid(row=10, column=1, sticky="e")

        # Buttons
        apply_btn = tk.Button(self, text="Apply", command=self.apply_changes)
        reset_btn = tk.Button(self, text="Reset", command=self.reset_changes)
        apply_btn.grid(row=11, column=1, pady=10)
        reset_btn.grid(row=11, column=2, pady=10)

        self.update_active_controller("PD")

    def update_active_controller(self, controller_type):
        is_pd = self.controller_type.get() == "PD"

        for entry in self.pd_entries:
            entry.config(state="normal" if is_pd else "disabled")
        for entry in self.G_entries:
            entry.config(state="normal" if not is_pd else "disabled")
        self.k_entry.config(state="normal" if not is_pd else "disabled")
        self.e_entry.config(state="normal" if not is_pd else "disabled")

    def apply_changes(self):
        if self.controller_type.get() == "PD":
            for i, field in enumerate(fields(self.pd_params)):
                try:
                    value = float(self.pd_entries_values[i].get())
                    setattr(self.pd_params, field.name, value)
                except ValueError:
                    print(f"Invalid PD value: {self.pd_entries_values[i].get()}")
        else:
            try:
                self.smc_params.G = [[float(self.G_vars[i][j].get()) for j in range(3)] for i in range(3)]
                self.smc_params.k = float(self.k_var.get())
                self.smc_params.e = float(self.e_var.get())
            except ValueError as err:
                print(f"Invalid SMC parameter: {err}")

    def reset_changes(self):
        if self.controller_type.get() == "PD":
            self.pd_params = PDParameters()
            for i, field in enumerate(fields(self.pd_params)):
                self.pd_entries_values[i].set(str(getattr(self.pd_params, field.name)))
        else:
            self.smc_params = SMCParameters()
            for i in range(3):
                for j in range(3):
                    self.G_vars[i][j].set(str(self.smc_params.G[i][j]))
            self.k_var.set(str(self.smc_params.k))
            self.e_var.set(str(self.smc_params.e))


class AboutPage(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        label = ttk.Label(self, text="About This App")
        label.pack(pady=20)


class SimulationPage(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.params = AnimationParameters()

        self.t0 = tk.StringVar()
        self.t = tk.StringVar()
        self.dt = tk.StringVar()
        self.interval = tk.StringVar()
        self.ani_filepath = tk.StringVar()
        self.plt_filepath = tk.StringVar()
        self.str_vars = [self.t0, self.t, self.dt, self.interval, 
                         self.ani_filepath, self.plt_filepath]

        tk.Label(self, text="Start time (s):").grid(row=1, column=0, sticky="w")
        tk.Label(self, text="End time (s):").grid(row=2, column=0, sticky="w")
        tk.Label(self, text="Time step (s):").grid(row=3, column=0, sticky="w")
        tk.Label(self, text="Refresh rate (ms):").grid(row=4, column=0, sticky="w")
        tk.Label(self, text="Animation filepath:").grid(row=5, column=0, sticky="w")
        tk.Label(self, text="Plot filepath").grid(row=6, column=0, sticky="w")

        e_t0 = tk.Entry(self, textvariable=self.t0, width=5)
        e_t0.grid(row=1, column=1, sticky="w")

        e_t = tk.Entry(self, textvariable=self.t, width=5)
        e_t.grid(row=2, column=1, sticky="w")

        e_dt = tk.Entry(self, textvariable=self.dt, width=5)
        e_dt.grid(row=3, column=1, sticky="w")

        e_interval = tk.Entry(self, textvariable=self.interval, width=5)
        e_interval.grid(row=4, column=1, sticky="w")

        e_animation_filepath = tk.Entry(self, textvariable=self.ani_filepath, width=30)
        e_animation_filepath.grid(row=5, column=1, sticky="w")

        e_plt_path = tk.Entry(self, textvariable=self.plt_filepath, width=30)
        e_plt_path.grid(row=6, column=1, sticky="w")

        apply_btn = tk.Button(self, text="Apply", command=self.apply_changes)
        reset_btn = tk.Button(self, text="Reset", command=self.reset_changes)
        apply_btn.grid(row=11, column=1, pady=10)
        reset_btn.grid(row=11, column=2, pady=10)

        self.reset_changes()

    def apply_changes(self):
        pass

    def reset_changes(self):
        self.params = AnimationParameters()
        for i, x in enumerate(fields(self.params)):
            self.str_vars[i].set(str(getattr(self.params, x.name)))

if __name__ == "__main__":
    app = App()
    app.mainloop()
    app.quit()
