import tkinter as tk
from tkinter import filedialog
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from functools import partial
from typing import Optional

from frames.actuatorSystem_frame import ActuatorSystemFrame
from attitude import AttitudeAnimation
from frames.satellite_frame import SatelliteFrame
from misc import test_page


class SimulationFrame(tk.Frame):
    """
    A Tkinter frame responsible for hosting and controlling the attitude simulation.

    It integrates the Matplotlib 3D attitude animation, displays the current 
    satellite/reference state values, and provides a control bar for simulation 
    management (start, stop, reset, save, etc.).
    """
    def __init__(self, 
                 parent,
                 anim_obj: Optional[AttitudeAnimation]=None):
        """
        Initializes the SimulationFrame.

        Args:
            parent: The parent widget (Tkinter container).
            anim_obj (Optional[AttitudeAnimation]): The AttitudeAnimation object 
                                                    to display and control. If None, 
                                                    a new one is created.
        """
        super().__init__(parent)


        controls = tk.Frame(self)
        controls.pack(side="bottom", fill="x")

        # Main vertical split: top (plots) + bottom (controls)
        top_frame = tk.Frame(self)
        top_frame.pack(side="top", fill="both", expand=True)

        # Left side (attitude simulation)
        # self.anim_obj = AttitudeSimulation() if anim_obj is None else anim_obj
        self.anim_obj = AttitudeAnimation() if anim_obj is None else anim_obj
        self.canvas = FigureCanvasTkAgg(self.anim_obj.fig, master=top_frame)
        self.canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

        self.create_anim()

        right_frame = tk.Frame(top_frame)
        right_frame.pack(side="right", fill="both", anchor="w")

        sat_frame = tk.Frame(right_frame)
        sat_frame.pack(side="top", fill="both", anchor="w")
        self.sat_comp_frame = SatelliteFrame(sat_frame, self.anim_obj)
        self.sat_comp_frame.pack()

        # Right side (actuator plots depreciated. Just using values)
        actuator_frame = tk.Frame(right_frame)
        actuator_frame.pack(side="top", fill="both", expand=True)
        self.act_sys_frame = ActuatorSystemFrame(actuator_frame, self.anim_obj.sat.actuator_system)
        self.act_sys_frame.pack(fill="both", expand=True)
        # ActuatorSystemFrame(actuator_frame, self.anim_obj.sat.actuator_system).pack(fill="both", expand=True)

        # Bottom (control bar)
        # self.animator = Animator(self.anim_obj.sat.actuator_system.animations)
        # self.animator = Animator([self.anim_obj])
        # self.animator = Animator([self.anim_obj] + self.anim_obj.sat.actuator_system.animations)
        self.draw_control_bar(controls)

        self.running = True
        # print(self.anim_obj.sat.state)
        # print(self.anim_obj.ref.state)
        # print(self.anim_obj.controller)

    def create_anim(self):
        """
        Initializes the Matplotlib FuncAnimation object for the simulation plot.
        Sets the animation to paused initially.
        """
        self.animation = FuncAnimation(
            self.anim_obj.fig,
            partial(self.update_anim),
            frames=self.anim_obj.t.size,
            interval=self.anim_obj.time.interval,
            init_func=self.init_anim,
            # blit=False
            blit=True
        )
        self.anim_obj.fig.tight_layout()
        self.animation.pause()
        self.pause = True

    def init_anim(self):
        """
        Calls the initialization function for the attitude animation.

        Returns:
            The initial graphics objects for blitting.
        """
        return self.anim_obj.init_anim()

    def update_anim(self, frame):
        """
        The main update function called by FuncAnimation for each frame.

        It advances the simulation state and updates the display frames
        for actuator system and satellite state comparison.

        Args:
            frame: The current frame number (time step index).

        Returns:
            The updated graphics objects for blitting.
        """
        output = self.anim_obj.update_anim(frame)
        self.act_sys_frame.update_values()
        self.sat_comp_frame.update_values()
        return output

    def start_anim(self):
        """
        Resumes the Matplotlib animation.
        """
        self.pause = False
        self.animation.resume()

    def stop_anim(self):
        """
        Pauses the Matplotlib animation.
        """
        self.pause = True
        self.animation.pause()

    def reset(self):
        """
        Resets the simulation to the initial state, updates the control frames,
        stops the animation, and redraws the initial plot.
        """
        self.anim_obj.reset_anim()
        self.act_sys_frame.update()

        self.stop_anim()
        self.init_anim()
        self.update_anim(0)
        self.anim_obj.fig.canvas.draw()

    def step(self):
        """
        Performs a single step of the simulation (moves to the next frame), 
        updates the control frames, and redraws the plot.
        """
        self.act_sys_frame.update()
        self.update_anim(0)
        self.anim_obj.fig.canvas.draw()

    def draw_control_bar(self, frame: tk.Frame):
        """
        Draws the control buttons (Start, Stop, Reset, Step, Randomize, Reference, Save) 
        at the bottom of the frame.

        Args:
            frame (tk.Frame): The Tkinter frame to place the controls in.
        """
        self.start_btn = tk.Button(frame, text="Start", command=lambda: self.start_anim())
        # self.start_btn = tk.Button(frame, text="Start", command=self.anim_obj.start_anim)
        # self.start_btn = tk.Button(frame, text="Start", command=self.animator.start)
        self.start_btn.pack(side=tk.LEFT)

        self.stop_btn = tk.Button(frame, text="Stop", command=lambda: self.stop_anim())
        # self.stop_btn = tk.Button(frame, text="Stop", command=self.anim_obj.stop_anim)
        # self.stop_btn = tk.Button(frame, text="Stop", command=self.animator.stop)
        self.stop_btn.pack(side=tk.LEFT)

        self.reset_btn = tk.Button(frame, text="Reset", command=lambda: self.reset())
        # self.reset_btn = tk.Button(frame, text="Reset", command=self.anim_obj.reset_anim)
        # self.reset_btn = tk.Button(frame, text="Reset", command=self.animator.reset)
        self.reset_btn.pack(side=tk.LEFT)

        self.step_btn = tk.Button(frame, text="Step", command=lambda: self.step())
        self.step_btn.pack(side=tk.LEFT)

        self.randomize_btn = tk.Button(frame, text="Randomize", command=self.randomize)
        self.randomize_btn.pack(side=tk.LEFT)

        self.reference = tk.IntVar(value=1)
        self.reference_btn = tk.Checkbutton(frame,
                                            text="Reference",
                                            variable=self.reference,
                                            onvalue=1,
                                            offvalue=0,
                                            command=self.set_ref)
        self.reference_btn.pack(side=tk.LEFT)

        self.file_path: str = ""
        self.save_as_btn = tk.Button(frame,
                                     text="Save as",
                                     command=self.save_as)
        self.save_btn = tk.Button(frame,
                                     text="Save",
                                     command=self.save)
        self.save_btn.pack(side=tk.RIGHT)
        self.save_as_btn.pack(side=tk.RIGHT)

    def save_as(self):
        """
        Prompts the user for a file path to save the animation and then calls save().
        """
        file_path: str = filedialog.asksaveasfilename()
        if file_path == "":
            return

        self.file_path = file_path
        self.save()

    def save(self):
        """
        Saves the animation to the determined file path. If no path is set, 
        it first prompts the user via save_as().
        """
        if self.file_path == "":
            return self.save_as()
        # self.anim_obj.animation.save(self.file_path, 
        self.animation.save(self.file_path, 
                            writer="pillow",
                            fps=int(1000/self.anim_obj.time.interval))
                            # fps=int(1000/self.anim_obj.parameters_animation.interval))


    def set_ref(self):
        """
        Toggles the drawing of the reference object in the attitude plot based on 
        the 'Reference' checkbutton state.
        """
        self.anim_obj.draw_reference = bool(self.reference.get())
        self.anim_obj.flush_plot()
        self.canvas.draw_idle()
        self.anim_obj.init_anim()
        self.anim_obj.draw_satellite()
        if self.anim_obj.draw_reference:
            self.anim_obj.draw_ref()
        self.canvas.draw_idle()


    def randomize(self):
        """
        Randomizes the initial attitude of both the satellite and the reference 
        and resets the simulation.
        """
        self.anim_obj.sat.state.randomize_attitude()
        self.anim_obj.ref.state.randomize_attitude()
        self.reset()
        # self.anim_obj.reset_anim()
        # self.animator.reset()



def test_desaturation():
    """
    A test function to run the SimulationFrame with pre-set reaction wheel speeds 
    to test desaturation logic.
    """
    root = tk.Tk()
    sim_frame = SimulationFrame(root)
    for rw in sim_frame.anim_obj.sat.actuator_system.reaction_wheels:
        rw.w = 10
    test_page(root, sim_frame)

if __name__ == '__main__':
    # root = tk.Tk()
    # test_page(root, SimulationFrame(root))
    # test_page(root, SimulationFrame(root, create_simulation()))
    test_desaturation()
