import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass, field
from typing import List
from enum import Enum


class Plot2D(Enum):
    """
        The supported 2D plots to accomodate the 3D
        visualization.
    """
    TORQUE = 0
    ANGULAR_VELOCITY = 1
    QUTERNION = 2


@dataclass
class AnimationParameters:
    T0: float = 0.0
    T_stop: float = 20.0 # End time of the save gif of the animation
    dt: float = 0.05
    interval: int = 5 # Refresh rate for the animation in ms
    reference: bool = True
    animation_file_path: str = "Simulations/animation.gif"
    plot_file_path: str = "Plots/simulation.pdf"


@dataclass
class PlotParameters:
    reference: bool = True
    alpha: float = 0.7
    width: int = 6          # Inches
    height: int = 4         # Inches
    # width: int = 800
    # height: int = 600
    colors: List[str] = field(default_factory=lambda: ["red", "blue", "green"])


@dataclass
class InitialParameters:
    Q: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    W: List[float] = field(default_factory=lambda: [0.3, 0.3, 0.0])
    W_dot: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class ReferenceParameters:
    rot: Rotation = field(default_factory=
                          lambda: Rotation.from_quat([1.0, 0.0, 0.0, 0.0]))
    W: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    W_dot: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))

    theta: float = 90.0
    phi: float = 90.0


@dataclass
class PDParameters:
    P_x: float = 1.0
    P_y: float = 1.0
    P_z: float = 1.0

    D_x: float = 1.0
    D_y: float = 1.0
    D_z: float = 1.0


@dataclass
class SMCParameters:
    G: List[List[float]] = field(default_factory=lambda:[[0.15, 0.0, 0.0],
                                                         [0.0, 0.15, 0.0],
                                                         [0.0, 0.0, 0.15]])
    k: float = 0.015
    e: float = 0.01

