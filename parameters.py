from dataclasses import dataclass, fields

@dataclass
class SimulationParameters:
    pass

@dataclass
class AnimationParameters:
    T0: float = 0.0
    T_stop: float = 20.0 # End time of the save gif of the animation
    dt: float = 0.1
    interval: int = 5 # Referesh rate for the animation in ms
    animation_file_path: str = "Simulations/animation.gif"
    plot_file_path: str = "Plots/simulation.pdf"

@dataclass
class InitialParameters:
    Q = [1.0, 0.0, 0.0, 0.0]
    W = [0.3, 0.3, 0.0]
    W_dot = [0.0, 0.0, 0.0]

@dataclass
class ReferenceParameters:
    Q = [1.0, 0.0, 0.0, 0.0]
    W = [0.3, 0.3, 0.0]
    W_dot = [0.0, 0.0, 0.0]

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
    G = [[0.15, 0.0, 0.0],
         [0.0, 0.15, 0.0],
         [0.0, 0.0, 0.15]]
    k: float = 0.015
    e: float = 0.01

