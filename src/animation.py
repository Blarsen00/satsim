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
    t0: float = 0.0         # Start time for the save
    t_end: float = 20.0     # End time for the save
    dt: float = 0.05        # dt for the simulation
    interval: int = 40      # Refresh rate for the animation in ms


class BaseAnimation(ABC):
    time: TimeParameters = TimeParameters()

    def __init__(self) -> None:
        self.t = np.arange(self.time.t0, self.time.t_end, self.time.dt)
        self.display_vars = {}
        self.key = "torque"

        # WARN: Plotting is currently not functioning as the feature did not work as 
        # intended, and might be introduced again at some point. Currently removed, 
        # and replaced by displaying the most recent value instead.

        # # WARN: Needs to create self.fig before calling the init super()__init__(), 
        # # as this init function depends on it. See AttitudeAnimation for reference
        # self.create_anim()

    # WARN: Currently depreciated
    # def create_anim(self):
    #     self.animation = FuncAnimation(
    #         self.fig,
    #         partial(self.update_anim),
    #         frames=self.t.size,
    #         interval=self.time.interval,
    #         init_func=self.init_anim,
    #         # blit=False
    #         blit=True
    #     )
    #     self.fig.tight_layout()
    #     self.animation.pause()
    #     self.pause = True

    def load_time_parameters(self, time: TimeParameters):
        self.time = time
        self.t = np.arange(self.time.t0, self.time.t_end, self.time.dt)

    # WARN: Currently depreciated
    # def start_anim(self):
    #     self.pause = False
    #     self.animation.resume()

    # WARN: Currently depreciated
    # def stop_anim(self):
    #     self.pause = True
    #     self.animation.pause()
    #     # self.create_anim()

    # WARN: Currently depreciated
    # def reset_anim(self):
    #     self.stop_anim()
    #     self.init_anim()
    #     self.update_anim(0)
    #     self.fig.canvas.draw()

    @abstractmethod
    def draw(self):
        pass

    @abstractmethod
    def init_anim(self) -> Iterable[Artist]:
        pass

    @abstractmethod
    def update_anim(self, frame: int) -> Iterable[Artist]:
        pass


class Animator:
    def __init__(self, animations:Sequence[BaseAnimation]) -> None:
        self.animations: List[BaseAnimation] = list(animations)

    def add_animation(self, anim: BaseAnimation):
        self.animations.append(anim)

    def stop(self):
        for anim in self.animations:
            anim.stop_anim()

    def start(self):
        for anim in self.animations:
            anim.start_anim()

    def reset(self):
        for anim in self.animations:
            anim.reset_anim()

