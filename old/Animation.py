import numpy as np
import quaternion
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
from functools import partial

from Satellite import Satellite
from Reference import WMAP, SimplePointing
from Reference import Reference, EarthReferencePD
from Orbit import Orbit
from Controllers import Controller
from Plot import Plot
import yaml


class AttitudeAnimation:
    """
    Class for animating the satellite movements. Computes and stores the data for:
        - Attitude -> Quaternion
        - Angular velocity -> W
        - Applied torque -> Tau
    """

    def __init__(self, 
                 satellite: Satellite,
                 controller: Controller,
                 reference: Reference,
                 plots,
                 plot3d="Yaml/3dplot.yaml",
                 simulation="Yaml/Simulation.yaml"):

        self.satellite = satellite
        self.reference = reference
        self.controller = controller
        self.R0 = quaternion.as_rotation_matrix(self.satellite.q)

        ##################################################
        ############# Simulation parameters ##############
        ##################################################

        with open(simulation, 'r') as file:
            self.sim = yaml.safe_load(file)
        file.close()

        # Animation parameters
        self.t0 = self.sim["t0"]
        self.t = self.sim["t"]
        self.dt = self.sim["dt"]
        self.interval = self.sim["interval"]
        self.time = np.arange(self.t0, self.t, self.dt)
        self.fig_width = self.sim["fig_width"]
        self.fig_height = self.sim["fig_height"]

        try:
            self.file_name = self.sim["file_name"]
            self.animation = True
        except:
            print(f"No path is specified. Animation will not be saved")
            self.animation = False

        ##################################################
        ################## Other plots ###################
        ##################################################

        # Make dictionaries to store the plot object, axis and lines for the other plots.
        # The name of the plot is the key, say "rpm" or "quaternion". 
        # The dictionaries are called plots, other_axes and other_lines
        try:
            tmp = [Plot(plot) for plot in plots]
            self.plots = {}
            self.other_axes = {}
            self.other_lines = {}
            self.other_references = {}
            self.plot_size = 0
            for plot in tmp:
                self.plots[plot.name] = plot
                self.other_axes[plot.name] = []
                self.other_lines[plot.name] = []
                self.other_references[plot.name] = []
                self.plot_size += plot.size
        except:
            print(f"The listed plots could not be resolved.")

        # Initialize the figure
        self.fig = plt.figure(figsize=(self.fig_width, self.fig_height))
        self.layout = [self.plot_size, 2]
        self.gs = gridspec.GridSpec(*self.layout, figure=self.fig)

        # Assign the other plots to the gridspec and add the lines and axes to 
        # the dictionaries specified above
        row = 0
        for plot in tmp:
            for i in range(plot.size):
                ax = self.fig.add_subplot(self.gs[row,1])

                ax.grid(plot.grid)
                ax.set_xlim([self.t0, self.t])
                ax.set_ylim(plot.ylim[i % len(plot.ylim)])
                ax.set_ylabel(rf"{plot.ylables[i % len(plot.ylables)]}")

                ln, = ax.plot([], [], plot.colors[i % len(plot.colors)])
                lnref, = ax.plot([], [], f"{plot.colors[i % len(plot.colors)][0]}--")

                self.other_axes[plot.name].append(ax)
                self.other_lines[plot.name].append(ln)
                self.other_references[plot.name].append(lnref)
                row += 1

        # Set only the last plot to have xlabel
        ax.set_xlabel("Time(s)")

        ##################################################
        #################### 3d Plot #####################
        ##################################################

        with open(plot3d, 'r') as file:
            self.plot3d = yaml.safe_load(file)
        file.close()

        # Add the text box to display quaternion and reference quaternion to the figure
        self.axtxt = self.fig.add_subplot(self.gs[0,0])
        self.axtxt.axis('off')
        self.text_box = self.axtxt.text(0.3, 0.5, '', 
                                        fontsize=12, 
                                        bbox=dict(facecolor='gray', 
                                                  edgecolor='black',
                                                  boxstyle='round,pad=0.5'),
                                        transform=self.axtxt.transAxes,
                                        ha='left', va='center')       #boxstyle='round,pad=0.3'))


        # Load the limits of the plot
        self.xlim_3d = self.plot3d["xlim"]
        self.ylim_3d = self.plot3d["ylim"]
        self.zlim_3d = self.plot3d["zlim"]

        # Add the 3d plot to the figure
        self.ax3d = self.fig.add_subplot(self.gs[1:,0], projection='3d')

        # Plot the origin
        self.ax3d.scatter(0.0, 0.0, 0.0, 'black')

        # Plot the axis colors
        try:
            self.ax_colors = self.plot3d["axis colors"]
        except:
            self.ax_colors = ["green", "red", "blue"]
        self.set_axis_colors()

        ##################################################
        ############# Table to store data ###############
        ##################################################

        # Data for all of the other plots. Of the size [T, x+1] where x is the number 
        # of other plots. Say we have rpm and quaternion, each consisting of 3 and 4 plots
        # which would mean x has the size of 3+4=7. The last column will be the time.
        self.data = np.zeros([int((self.t - self.t0) / self.dt), self.plot_size])
        self.reference_data = np.zeros_like(self.data)

        print()
        print(f"DATA: {self.data.shape}")
        print(f"TIME: {self.time.shape}")


        # Create the animation object
        self.ani = FuncAnimation(
            self.fig, 
            partial(self.update), 
            frames=len(self.time), 
            interval=self.interval,
            init_func=self.init, 
            blit=True
        )
        self.fig.tight_layout()


    def set_axis_colors(self):
        """
        Set the axis colors of the 
        """
        # Plot the axis default axis colors
        self.x_axis, = self.ax3d.plot([], [], [], color=self.ax_colors[0])
        self.y_axis, = self.ax3d.plot([], [], [], color=self.ax_colors[1 % len(self.ax_colors)])
        self.z_axis, = self.ax3d.plot([], [], [], color=self.ax_colors[2 % len(self.ax_colors)])

        # Plot the reference axis with an opacity
        self.ref_x_axis, = self.ax3d.plot([], [], [], alpha=0.3, color=self.ax_colors[4 % len(self.ax_colors)])
        self.ref_y_axis, = self.ax3d.plot([], [], [], alpha=0.3, color=self.ax_colors[5 % len(self.ax_colors)])
        self.ref_z_axis, = self.ax3d.plot([], [], [], alpha=0.3, color=self.ax_colors[6 % len(self.ax_colors)])

    def draw_satellite(self):
        R = quaternion.as_rotation_matrix(self.satellite.q) 
        RC = quaternion.as_rotation_matrix(self.reference.qc) 

        # Draw the satellite axes
        self.x_axis.set_data([0.0, R[0,0]], [0.0, R[1,0]])
        self.x_axis.set_3d_properties([0.0, R[2,0]])      # Set z-axis properties

        self.y_axis.set_data([0.0, R[0,1]], [0.0, R[1,1]])
        self.y_axis.set_3d_properties([0.0, R[2,1]])      # Set z-axis properties

        self.z_axis.set_data([0.0, R[0,2]], [0.0, R[1,2]])
        self.z_axis.set_3d_properties([0.0, R[2,2]])      # Set z-axis properties

        self.ref_x_axis.set_data([0.0, RC[0,0]], [0.0, RC[1,0]])
        self.ref_x_axis.set_3d_properties([0.0, RC[2,0]])      # Set z-axis properties

        self.ref_y_axis.set_data([0.0, RC[0,1]], [0.0, RC[1,1]])
        self.ref_y_axis.set_3d_properties([0.0, RC[2,1]])      # Set z-axis properties

        self.ref_z_axis.set_data([0.0, RC[0,2]], [0.0, RC[1,2]])
        self.ref_z_axis.set_3d_properties([0.0, RC[2,2]])      # Set z-axis properties


    def init(self):
        # Load the limits of the plot
        self.xlim_3d = self.plot3d["xlim"]
        self.ylim_3d = self.plot3d["ylim"]
        self.zlim_3d = self.plot3d["zlim"]

        self.ax3d.set_xlim(self.xlim_3d)
        self.ax3d.set_ylim(self.ylim_3d)
        self.ax3d.set_zlim(self.zlim_3d)

        # Set the labes of the plot
        self.ax3d.set_xlabel("x-axis")
        self.ax3d.set_ylabel("y-axis")
        self.ax3d.set_zlabel("z-axis")

        # Draw the 3d axes of the satellite
        self.draw_satellite()

        self.text_box.set_text(f"Q: {quaternion.as_float_array(self.satellite.q)} \
                               \n QC: {quaternion.as_float_array(self.reference.qc)}")

        lines = [self.x_axis, self.y_axis, self.z_axis]
        for key in self.other_axes:
            for ax in self.other_axes[key]:
                lines += ax.get_lines()
        return lines


    def update(self, frame):
        # self.reference.update(dt=self.dt * 20)
        self.reference.update(dt=self.dt)
        L = self.controller.output(self.satellite, self.reference)
        self.satellite.update_attitude(L=L, dt=self.dt)

        # Store and set the calculated data
        col = 0
        for key in self.plots:
            plot = self.plots[key]
            data = self.satellite.get_data(plot.name)
            if plot.name == "torque":
                ref = L
            else:
                ref = self.reference.get_data(plot.name)
                # print(ref)
            for i in range(plot.size):
                self.data[frame, col] = data[i]
                self.other_lines[key][i].set_ydata(self.data[:frame+1, col])
                self.other_lines[key][i].set_xdata(self.time[:frame+1])

                self.reference_data[frame, col] = ref[i]
                self.other_references[key][i].set_ydata(self.reference_data[:frame+1, col])
                self.other_references[key][i].set_xdata(self.time[:frame+1])
                col += 1

        # Draw the 3d axes of the satellite
        self.draw_satellite()

        # Display the update quaternion values
        self.text_box.set_text(f"Q: {np.round(self.satellite.get_data('quaternion'), 3)} \n"
            f"QC: {np.round(quaternion.as_float_array(self.reference.qc), 3)}"
        )

        # Create a list of all the line elements
        lines = [self.x_axis, self.y_axis, self.z_axis, 
                 self.ref_x_axis, self.ref_y_axis, self.ref_z_axis, 
                 self.text_box]

        # Add the lines of the other plots
        for key in self.other_axes:
            for ax in self.other_axes[key]:
                lines += ax.get_lines()

        return lines

if __name__ == "__main__":
    sat = Satellite()
    controller = Controller()
    ref = Reference()
    orbit = Orbit()
    smc_ref = SimplePointing()
    earth_ref = EarthReferencePD(sat, orbit)
    wmap_ref = WMAP()
    controller.output(sat, earth_ref)
    plot = AttitudeAnimation(satellite=sat, 
                             controller=controller, 
                             reference=ref,
                             # reference=earth_ref,
                             # reference=wmap_ref,
                             # reference=smc_ref,
                             # plots=["Yaml/Quaternion.yaml"])
                             # plots=["Yaml/Torque.yaml"])
                             plots=["Yaml/Velocity.yaml"])
                             # plots=["Yaml/Velocity.yaml", "Yaml/Torque.yaml"])
                             # plots=["Yaml/velocity.yaml", "Yaml/Quaternion.yaml"])
                             # plots=["Yaml/Torque.yaml", "Yaml/Quaternion.yaml"])
    # if plot.animation:
    #     plot.ani.save(plot.file_name, writer="pillow", fps=30)
    plt.show()
