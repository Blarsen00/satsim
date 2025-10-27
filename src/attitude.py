import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

from typing import Optional

from satellite import Satellite
from reference import BaseReference
from controller import Controller, PDController
from simulation import PhysicalState, Simulate
from animation import BaseAnimation


class AttitudeAnimation(BaseAnimation):
    """
    A class to create a 3D visualization of the attitude control simulation.

    This class inherits from :class:`~animation.BaseAnimation` and implements
    the specific methods for drawing the satellite and reference attitude axes
    and updating the simulation state frame by frame.

    Parameters
    ----------
    sat : :class:`~satellite.Satellite`, optional
        The satellite model being simulated. Defaults to a new :class:`Satellite` instance.
    ref : :class:`~reference.BaseReference`, optional
        The reference attitude generator. Defaults to a new :class:`BaseReference` instance.
    controller : :class:`~controller.Controller`, optional
        The controller used to generate torque. Defaults to a new :class:`~controller.PDController` instance.

    Attributes
    ----------
    sat : :class:`~satellite.Satellite`
        The satellite model.
    ref : :class:`~reference.BaseReference`
        The reference generator.
    controller : :class:`~controller.Controller`
        The attitude controller.
    fig : :class:`matplotlib.figure.Figure`
        The Matplotlib figure object.
    ax : :class:`matplotlib.axes.Axes`
        The 3D axes subplot.
    colors : list of str
        Colors used for the X, Y, Z axes.
    alpha : float
        Transparency level for the reference axes.
    draw_reference : bool
        Flag to determine if the reference axes should be drawn.
    axes : list of :class:`matplotlib.artist.Artist`
        List of 3D line artists representing the satellite's body axes.
    axes_ref : list of :class:`matplotlib.artist.Artist`
        List of 3D line artists representing the reference body axes.
    """
    def __init__(self,
                 sat:Optional[Satellite]=None,
                 ref: Optional[BaseReference]=None,
                 controller: Optional[Controller]=None) -> None:

        self.sat: Satellite = Satellite() if sat is None else sat
        self.ref: BaseReference = BaseReference() if ref is None else ref
        self.controller: Controller = PDController() if controller is None else controller

        # Plotting axes setup
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.scatter(0.0, 0.0, 0.0, color='black')

        self.colors = ["red", "green", "blue"]
        self.alpha = 0.6
        self.draw_reference = True

        super().__init__()

    def load_controller_parameters(self, parameters=None):
        # WARN: This function is probably not used at all and can possible be removed
        """
        Placeholder method (assuming it's intended to be implemented in subclasses
        or the controller itself) to load new parameters into the controller.

        Parameters
        ----------
        parameters : dict, optional
            A dictionary of controller parameters to load. Defaults to None.
        """
        self.controller.load_params(parameters)

    def init_anim(self):
        """
        Initializes the 3D plot artists (lines) for the satellite and reference
        axes. This is used as the `init_func` for :class:`~matplotlib.animation.FuncAnimation`.

        Returns
        -------
        list of :class:`matplotlib.artist.Artist`
            The combined list of satellite and reference axes artists.
        """
        self.axes = []
        self.axes_ref = []
        for i in range(3):
            # Satellite axes
            (line, ) = self.ax.plot(
                                 [],
                                 [],
                                 [],
                                 self.colors[i % len(self.colors)])
            self.axes.append(line)

            # Reference axes (with alpha for distinction)
            (line_ref, ) = self.ax.plot(
                                 [],
                                 [],
                                 [],
                                 self.colors[
                                 i % len(self.colors)],
                                 alpha=self.alpha)
            self.axes_ref.append(line_ref)

        # Set fixed limits for the 3D plot
        self.ax.set_xlim((-1.0, 1.0))
        self.ax.set_ylim((-1.0, 1.0))
        self.ax.set_zlim((-1.0, 1.0))

        # Set the labes of the plot
        self.ax.set_xlabel("x-axis")
        self.ax.set_ylabel("y-axis")
        self.ax.set_zlabel("z-axis")

        return self.axes + self.axes_ref

    def draw_ref(self):
        """
        Updates the 3D line artists to represent the current reference attitude.
        """
        R = self.ref.state.rot.as_matrix()
        for i in range(3):
            self.axes_ref[i].set_data([0.0, R[0, i]], [0.0, R[1, i]])
            self.axes_ref[i].set_3d_properties([0.0, R[2, i]])


    def draw_satellite(self):
        """
        Updates the 3D line artists to represent the current satellite attitude.
        """
        R = self.sat.state.rot.as_matrix()
        for i in range(3):
            self.axes[i].set_data([0.0, R[0, i]], [0.0, R[1, i]])
            self.axes[i].set_3d_properties([0.0, R[2, i]])

    def draw(self):
        """
        Draws the satellite and reference axes (if enabled) at the current state.
        """
        self.draw_satellite()
        if self.draw_reference:
            self.draw_ref()
        # Calls BaseAnimation.draw() if it performs additional actions, otherwise redundant
        # return super().draw() # Retaining the original call for compatibility
        return

    def update_anim(self, frame, dt=0.1):
        """
        Performs one simulation step, updates the satellite and reference states,
        and redraws the visualization.

        This is used as the update function for :class:`~matplotlib.animation.FuncAnimation`.

        Parameters
        ----------
        frame : int
            The current frame index (not directly used for time).
        dt : float, optional
            The time step for integration (overridden by ``self.time.dt`` in implementation).
            Defaults to 0.1 (Though the implementation uses ``self.time.dt``).

        Returns
        -------
        list of :class:`matplotlib.artist.Artist`
            The combined list of updated satellite and reference axes artists.
        """
        u = self.controller.output(self.sat.state, self.ref.state, J=self.sat.J)
        L_applied = self.sat.actuator_system.apply_torque(u,
                                                         b0=self.sat.b0,
                                                         R=self.sat.state.rot.as_matrix(),
                                                         kp=2.0,
                                                         w=self.sat.state.w,
                                                         b=self.sat.get_magnetic_field_vector())
        self.sat.state = Simulate.update_state(L_applied,
                                               self.sat.state,
                                               self.sat.J,
                                               self.time.dt)
        self.ref.update(self.time.dt)

        self.draw()
        return self.axes + self.axes_ref

    def reset_anim(self):
        """
        Resets the plot artists to be empty and resets the satellite state to its initial configuration.
        """
        self.flush_plot()
        self.sat.reset()
        # super().reset_anim() # Retaining the original call for compatibility

    def flush_plot(self):
        """
        Clears all data from the satellite and reference axes line artists,
        effectively resetting the drawing state.
        """
        for line in self.axes:
            line.set_data([], [])
            line.set_3d_properties([])

        for line in self.axes_ref:
            line.set_data([], [])
            line.set_3d_properties([])


def test_parameters_sat():
    """
    Creates and returns a specific initial :class:`~satellite.Satellite`,
    :class:`~reference.BaseReference`, and :class:`~controller.PDController`
    for testing purposes.

    The satellite starts at identity attitude with zero angular velocity.
    The reference is set to a 90-degree rotation about the Z-axis.

    Returns
    -------
    sat : :class:`~satellite.Satellite`
        The initialized satellite object.
    ref : :class:`~reference.BaseReference`
        The initialized reference object.
    controller : :class:`~controller.Controller`
        The initialized PD controller object.
    """
    rot = Rotation.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)]) # 90 degree Z-rotation
    rot_I = Rotation.from_quat([0, 0, 0, 1.0])
    sat_param = PhysicalState(
        rot=rot_I,
        w=np.zeros(3),
        w_dot=np.zeros(3),
    )
    ref_param = PhysicalState(
        rot=rot,
        w=np.zeros(3),
        w_dot=np.zeros(3)
    )

    ref = BaseReference()
    ref.state = ref_param

    sat = Satellite()
    sat.load_satellite_parameters(sat_param)

    controller = PDController()
    return sat, ref, controller


def create_simulation():
    """
    Creates and returns a new :class:`AttitudeAnimation` instance using the
    default test parameters.

    Returns
    -------
    anim : :class:`AttitudeAnimation`
        The initialized animation object.
    """
    anim = AttitudeAnimation(*test_parameters_sat())
    return anim


def test_animation():
    """
    Sets up and runs a basic test of the attitude animation.
    Initial quaternion states are printed, and the Matplotlib figure is shown.
    """
    anim = create_simulation()
    print(f"Satellite quaternion: {anim.sat.state.rot.as_quat()}")
    print(f"Reference quaternion: {anim.ref.state.rot.as_quat()}")
    print("-----------------------------------------------------------------------")
    plt.show()


if __name__ == '__main__':
    test_animation()
