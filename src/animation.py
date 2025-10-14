from matplotlib.artist import Artist
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from functools import partial

from typing import List, Optional, Iterable, Sequence
from abc import abstractmethod, ABC
from dataclasses import dataclass


@dataclass
class TimeParameters:
    """
    Data class to hold time-related parameters for simulation and animation.

    Attributes
    ----------
    t0 : float, optional
        Start time for the simulation, in seconds. Defaults to 0.0.
    t_end : float, optional
        End time for the simulation, in seconds. Defaults to 20.0.
    dt : float, optional
        Time step (delta time) for the simulation/integration, in seconds.
        Defaults to 0.05.
    interval : int, optional
        Refresh rate for the :class:`matplotlib.animation.FuncAnimation`
        in milliseconds (ms). Defaults to 40.
    """
    t0: float = 0.0
    t_end: float = 20.0
    dt: float = 0.05
    interval: int = 40


class BaseAnimation(ABC):
    """
    Abstract base class for creating Matplotlib animations.

    This class provides the core structure, time management, and abstract
    methods required for subclasses to implement custom drawing and animation
    logic using :class:`matplotlib.animation.FuncAnimation`.

    Attributes
    ----------
    time : :class:`TimeParameters`
        Time parameters used to define the simulation timeline.
    t : :class:`numpy.ndarray`
        Array of time points from :attr:`time.t0` to :attr:`time.t_end`
        with step :attr:`time.dt`.
    display_vars : dict
        A dictionary intended to hold variables for display (e.g., current values).
    key : str
        A placeholder key, defaults to "torque".
    """
    time: TimeParameters = TimeParameters()

    def __init__(self) -> None:
        """
        Initializes the time array and internal state variables.
        """
        self.t = np.arange(self.time.t0, self.time.t_end, self.time.dt)
        self.display_vars = {}
        self.key = "torque"

        # WARN: Plotting is currently not functioning as the feature did not work as
        # intended, and might be introduced again at some point. Currently removed,
        # and replaced by displaying the most recent value instead.

        # # WARN: Needs to create self.fig before calling the init super()__init__(),
        # # as this init function depends on it. See AttitudeAnimation for reference
        # self.create_anim()

    def load_time_parameters(self, time: TimeParameters):
        """
        Updates the internal time parameters and regenerates the time array :attr:`t`.

        Parameters
        ----------
        time : :class:`TimeParameters`
            The new time parameters to be used.
        """
        self.time = time
        self.t = np.arange(self.time.t0, self.time.t_end, self.time.dt)

    @abstractmethod
    def draw(self):
        """
        Abstract method to be implemented by subclasses for drawing static
        elements of the visualization.
        """
        pass

    @abstractmethod
    def init_anim(self) -> Iterable[Artist]:
        """
        Abstract method that provides the initialization function for
        :class:`matplotlib.animation.FuncAnimation`.

        It should draw the background of a frame and return an iterable of
        the :class:`matplotlib.artist.Artist` objects that will be modified.

        Returns
        -------
        Iterable[:class:`matplotlib.artist.Artist`]
            Artists that are to be updated in each frame.
        """
        pass

    @abstractmethod
    def update_anim(self, frame: int) -> Iterable[Artist]:
        """
        Abstract method that provides the update function for
        :class:`matplotlib.animation.FuncAnimation`.

        This function is called sequentially for each frame of the animation.

        Parameters
        ----------
        frame : int
            The current frame number (index into :attr:`t`).

        Returns
        -------
        Iterable[:class:`matplotlib.artist.Artist`]
            Artists that have been updated in this frame.
        """
        pass

    # WARN: Currently depreciated
    # def create_anim(self):
    #     """
    #     (Deprecated) Creates and initializes the Matplotlib animation object.
    #     """
    #     # ... implementation details
    #     pass

    # WARN: Currently depreciated
    # def start_anim(self):
    #     """
    #     (Deprecated) Resumes the animation.
    #     """
    #     # ... implementation details
    #     pass

    # WARN: Currently depreciated
    # def stop_anim(self):
    #     """
    #     (Deprecated) Pauses the animation.
    #     """
    #     # ... implementation details
    #     pass

    # WARN: Currently depreciated
    # def reset_anim(self):
    #     """
    #     (Deprecated) Stops, resets the frame to 0, and redraws the initial state.
    #     """
    #     # ... implementation details
    #     pass


class Animator:
    """
    A controller class to manage a collection of :class:`BaseAnimation` instances.

    This class allows for coordinated starting, stopping, and resetting of
    multiple animations simultaneously.

    Parameters
    ----------
    animations : Sequence[:class:`BaseAnimation`]
        An initial sequence of :class:`BaseAnimation` objects to manage.

    Attributes
    ----------
    animations : List[:class:`BaseAnimation`]
        The list of managed animation objects.
    """
    def __init__(self, animations: Sequence[BaseAnimation]) -> None:
        self.animations: List[BaseAnimation] = list(animations)

    def add_animation(self, anim: BaseAnimation):
        """
        Adds a new animation instance to the manager's list.

        Parameters
        ----------
        anim : :class:`BaseAnimation`
            The animation object to add.
        """
        self.animations.append(anim)

    def stop(self):
        """
        Calls the (deprecated) ``stop_anim`` method on all managed animations.
        """
        for anim in self.animations:
            anim.stop_anim()

    def start(self):
        """
        Calls the (deprecated) ``start_anim`` method on all managed animations.
        """
        for anim in self.animations:
            anim.start_anim()

    def reset(self):
        """
        Calls the (deprecated) ``reset_anim`` method on all managed animations.
        """
        for anim in self.animations:
            anim.reset_anim()
