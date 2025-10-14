# Satellite Attitude Simulation

The following project is a simulation environment for satellite attitude and control logic. The program allows for testing of how different controllers and actuator setups affect the attitude control of a satellite. 


## Installation

Clone the repository: 

```
git clone https://github.com/Blarsen00/satsim
```

Setup the required environment, for example with a virtual environment: 

```
cd satsim
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

The virtual environment should now be active, and after going to the source folder `cd src`, the application can be ran with the following command: 
```
python3 -m app
```

# TODO 

1. Properly document the implemented code
    - Decide on a convention (Numpy docstring of Google style)
    - Properly document each and every class and function according to the chosen documentation
    - Use Sphinx to generate documentation from the docstrings
    - Delete obsolete files and code
    - Make an updated requirements.txt file
    - Add documentation about the satellite dynamics and physics in play.
    - Document the shortcomings of the application
        - Currently only true state estimates are fed into the controllers
        - No environmental disturbances are currently implemented
        - The integration scheme is direct Euler
2. Fix the double drawing issue where there are extra axis for the satellite being drawn
3. Fix the modifications to the actuator system
    - Changing the actuator systems in any way will currently freeze the dashboard in the simulation frame
    - The option button does not currently show a frame in which one can change the settings for the actuator in question
    - No way to add new magnetorquers to the actuator system at the moment
4. Add the reference for the pointing reference where each axis is pointing at two targets simultaneously and document the method properly.


## Satellite

The satellite is defined by its inertia matrix $J \in \mathbb{R}^{3\times 3}$, and it has its state defined by its attitude $q \in \mathbb{R}^{4}$ and angular velocity, $\omega \in \mathbb{R}^{3}$. The attitude is represented as a `Rotation` object from `scipy.spatial.transform`, which can represent the attitude as both a quaternion and a rotation matrix, among other useful features. 


```
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class PhysicalState:
    rot: Rotation = field(default_factory=lambda: Rotation.from_matrix(np.identity(3)))
    w: np.ndarray = field(default_factory=lambda: np.zeros(3))
    w_dot: np.ndarray = field(default_factory=lambda: np.zeros(3))
```


## Actuators

All actuators will need to have an axis of which it is pointing in the body frame, this is defined as `axis`. Each actuator will have a function that calculates the torque supplied by the actuator for a given reference torque `tau`. The function is called `apply_torque(self, tau: float, dt: float)`.

```
class Actuator:
    axis: np.ndarray = np.array([1.0, 0.0, 0.0])

    def __init__(self, axis: Optional[np.ndarray]=None) -> None:
        self.axis = np.array([1.0, 0.0, 0.0]) if axis is None else axis

    @abstractmethod
    def apply_torque(self, tau: float, dt: float=0.1):
```


## Reactionwheels

Reaction wheels are modeled where there are only two torques in play, the motor torque $\tau_m$ and the drag $\tau_d$. This means that the total torque provided by a reaction wheel is expressed as follows: 

$$
    \tau = \tau_m - \tau_d
$$

The drag $\tau_d$ is modeled as such:

$$
\tau_d = d_v \omega + d_c \text{sign}(\omega)
$$

where $d_v$ is the viscous damping coefficient, and $d_c$ is the Coloumb damping coefficient. The torque provided from the motor is assumed to be linear with the current $I$ provided to the motor, and is thus expressed as: 

$$
\tau_m = I k_m
$$

where $k_m$ is a positive constant. 

The reaction wheel is modeled om a way where the current available to the motors is limited. Controlling the torque from the reaction wheels will be controlling the current supplied to the motor. The transient response of the current to the engine is assumed to be instant and is thus disregarded, and the current is then modeled by a saturation function within the `Actuator` class as such: 

```
    @staticmethod
    def saturate(value: float,
                 maximum: Optional[float]=None,
                 minimum: Optional[float]=None):

        maximum = maximum if maximum is not None else np.infty
        minimum = minimum if minimum is not None else -np.infty

        return min(maximum, max(value, minimum))
```

This gives $-I_{max} \leq I \leq I_{max}$. The reaction wheels is otherwise considered to be perfect as of writing, but implementing other disturbances could be of interest at some point. With the current being the only limiting factor of the torque output of the motor, the response does seem fairly realistic. The current limit imposes a limit to the possible angular rate achievable by the reaction wheel as the wheel is unable to overcome the drag. This is expressed as follows.

$$
\begin{align}
    \tau &= I k_m - d_v \omega - d_c \text{sign}(\omega) \quad \quad \text{Set $\tau = 0$ and $\omega > 0$} \\
    0 &= I k_m - d_v \omega_{max} - d_c \\
    \omega_{max} &= \frac{I k_m - d_c}{d_v}
\end{align}
$$

The following plot shows how the torque produced by the reaction wheel relates to the angular velocity of the wheel. It is obvious that as the angular velocity of the wheel increases, the torque the wheel is able to produce diminishes. Once the angular rate matches the limit, then the torque approaches 0. The plot illustrates the importance of desaturation, as the reaction wheels operate at their most efficient, when their angular rate is kept low.

<!-- ![Torque test plot with max current set to 1.0 A](Simulation_files/torque_test_plot.pdf) -->
![Torque test plot with max current set to 1.0 A](https://github.com/Blarsen00/satsim/blob/main/Simulation_files/torque_test_plot.png)


## Magnetorquers

Magnetorquers produce torque based on its magnetic dipole $\mathbf{m}$, and the magnetic field vector $\mathbf{B}$ as follows: 

$$
    \tau = \mathbf{m} \times \mathbf{B}
$$

The magnetic dipole is expressed as follows: 

$$
    \mathbf{m} = n I \mathbf{A}
$$

Where $\mathbf{A}$ is the vector area of the coil, $n$ is the number of turns of the wire, and $I$ is the supplied current.

A proper implementation requires an implementation of Earths magnetic field, which is yet to be implemented, so in the meantime it is simply represented with a max torque limit where $-\tau_{max} \leq \tau \leq \tau_{max}$. 

