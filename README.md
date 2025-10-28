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

The dynamic equations in the body-fixed frame is as follows:
\begin{align}
    \nonumber J\dot\omega + \omega^\times (J\omega + h_\omega) &= -\tau_\omega + T_m \\
    \dot h_\omega &= \tau_w
\end{align}

Where $T_m$ is the magnetic torque produced by the magnetorquers, and $\tau_w$ is the torque provided by the reaction wheels.

The attitude kinematics of the satellite is modeled with the following differential equations:

$$
\begin{align}
    \nonumber
    \dot q = F(\omega) q &= \frac{1}{2} 
    \begin{bmatrix}
        -\omega^\times & \omega \\
        -\omega^T & 0 \\
    \end{bmatrix} q \\
    &= \frac{1}{2} 
    \begin{bmatrix}
        -\omega^\times\epsilon + \eta\omega \\
        -\omega^T\epsilon
    \end{bmatrix} q \\
\end{align}
$$

Runge-Kutta 4 is implemented to simulate the attitude dynamics of the satellite:
```
@staticmethod
def rk4(f: Callable[[np.ndarray, Union[float, np.floating]], np.ndarray],
        x: np.ndarray,
        t: Union[float, np.floating]=0.0,
        dt: Union[float, np.floating]=0.1):
    assert dt > 0.0
    x_k = x.copy()

    k1 = f(x_k, 0.0) * dt
    k2 = f(x_k + 0.5 * k1, t + 0.5 * dt) * dt
    k3 = f(x_k + 0.5 * k2, t + 0.5 * dt) * dt
    k4 = f(x_k + k3, t + dt) * dt

    weighted_sum = k1 + 2.0 * k2 + 2.0 * k3 + k4
    x_next = x_k + (1.0 / 6.0) * weighted_sum

    return x_next
```

The implementation calculates the angular velocity as follows: 
```
@staticmethod
def calculate_angular_accelleration(L: np.ndarray, w: np.ndarray, J: np.ndarray):
    J_inv = np.linalg.inv(J)
    w_dot = J_inv @ (L - misc.skew(w) @ J @ w)
    return w_dot

state.w = Simulate.rk4(
    f=lambda x, t: Simulate.calculate_angular_accelleration(L, x, J),
    x=state.w
)
```

and uses that calculated angular velocity to update the attitude as follows:
```
@staticmethod
def quaternion_derivate(q: np.ndarray, w: np.ndarray):
    qw = np.array([*w, 0])
    q_dot = 1/2 * misc.quat_multiply(q, qw)
    return q_dot

@staticmethod
def calculate_attitude(rot: Rotation, w: np.ndarray, dt: float):
    assert type(rot) is Rotation
    assert type(w) is np.ndarray and w.shape == (3,)
    assert dt > 0.0

    q = rot.as_quat()
    q_k = Simulate.rk4(
        f=lambda x, t: Simulate.quaternion_derivate(x, w),
        x=q
    )
    return Rotation.from_quat(q_k)
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

Where $\mathbf{A}$ is the vector area of the coil, $n$ is the number of turns of the wire, and $I$ is the supplied current. The current implementation of the magnetorquer models the magnetic dipole generated to be proportional to the current $I$, and a constant $k_m$, where $k_m = n A$, and the direction of the coil is set by the actuator attribute `self.axis`. 
The magnetic torque $T_m$ produced by the magnetorquers originates from the interaction between the local geomagnetic field $\tilde{b}$ and the magnetic momentum $\tau_m$ according to the following relationship $T_m = -\tilde{b}^\times \tau_m$. Then, the magnetic field $\tilde{b}$ is a function of both the location of the spacecraft along its orbit and its attitude. One can reqrite the expression, using $\tilde{b}_\circ$ as the geomagnetic filed expressed in the inertial frame:

$$
T_m = \Big(-R(q) \tilde{b}_{0}(t) \Big )^\times \tau_m
$$

Where $R(q)$ is the rotation matrix expressing the body-frame with respect to the inertial frame, whos expression depends on the attitude quaternion $q$.

The magnetic field in which the magnetorquers use to calculate the torque, is modeled as a constant vector in the inertial frame. The physical interpretation of such a model would be that the satellite is stationary in orbit, and thus the magnetic field vector points in the same direction relative to the inertial frame at all times. This is fine as there is yet to be implemented orbits for the satellite, so an implementation that uses the World Magnetic Model (WMM) does not make sense yet.

## Actuator System

The actuator system is a collection of independent actuators, and it is the actuator systems task to distribute torque to its actuators appropriately. The current implementation supports both reaction wheels and magnetorquers, which is the most prudent configuration, where the reaction wheels provide the attitude control, while the magnetorquers are tasked to desaturate the wheels. The class will have a list of actuators, `self.actuators`, a list of reaction wheels`self.reaction_wheels` and a list of magnetorquers `self.magnetorquers`.

```
self.actuators: Sequence[Actuator] = BASE_RW_CONFIG if actuators is None else actuators

# Divide the actuators into its sub components in order to better divide labor
self.magnetorquers: List[Magnetorquer] = [x for x in self.actuators if isinstance(x, Magnetorquer)]
self.reaction_wheels: List[ReactionWheel] = [x for x in self.actuators if isinstance(x, ReactionWheel)]
```
The actuator system implements a function to distribute torque called `apply_torque`.
The function takes a reference torque, `u`, provided by the controller and is the torque calculated to achieve pointing accuracy, and `**kwargs` which are the parameters needed for the desaturation of the reaction wheels. 
The function calculates the actuation inputs for the reaction wheels and magnetorquers and applies this torque to the best of the systems ability.

The desaturation logic uses the static input allocation method of Tregouet et al [^Tregouet_RW].
Equation 25 of the paper:

$$
\begin{align}
    \nonumber \tau_m &= - \frac{(R(q)\tilde{b}_\circ (t))^\times}{|\tilde{b}_\circ (t)|^2} k_p(h_w - h_{ref}) \\
    \tau_m^{[I]} &= - \frac{\tilde{b}_\circ^\times (t)}{|\tilde{b}_\circ (t)|^2} k_p(h_w^{[I]} - R^T(q)h_{ref})
\end{align}
$$

calculates the magnetic dipole for the magnetorquers. The body-fixed version is used.

```
def desaturate(self, b0: np.ndarray, R: np.ndarray, w_ref: float = 1.0, kp: float=1.0):
    h_ref = self.ref_angular_momentum_wheels(w_ref)
    B_body = R @ b0

    prelim_feedback = -misc.skew(B_body) / (np.linalg.norm(b0)**2)
    diff = kp * (self.angular_momentum_wheels() - h_ref)
    tau_m = prelim_feedback @ diff

    return tau_m
```

The reference angular momentum is calculated by summing the angular momentum for each of the reaction wheels with the reference angular velocity `w_ref` (rad/s).
`tau_m` is then distributed over the magnetorquers.
It is important to note that `tau_m` is the magnetic dipole that should be applied along each axis, and not the torque.
The actuator system makes a distribution matrix for the magnetorquers using their `axis` attribute as follows:

```
self.A_mqt = np.array([x.axis for x in self.magnetorquers]).T
self.A_mqt_inv = np.linalg.pinv(self.A_mqt)
```
Then `self.A_mqt_inv` is used to calculate the dipole that should be commanded by each magnetorquer.
This is all implemented in `distribute_torque_mqt` as follows:

```
def distribute_torque_mqt(self, tau_m: np.ndarray, b: np.ndarray) -> np.ndarray:
    L = np.zeros_like(tau_m)
    tau_A = self.A_mqt_inv @ tau_m
    for i, mqt in enumerate(self.magnetorquers):
        L[i] = mqt.apply_torque(tau_A[i])
    return L
```

This returns the applied dipole after accounting for the dynamics of the magnetorquers, which then is used to calculate the torque applied by the magnetorquers.

```
m_vec = self.distribute_torque_mqt(tau_m, b=B_body)
T_m = np.cross(m_vec, B_body)
```

Where `T_m` is the same as $T_m$. 
The desaturation paper [^Tregouet_RW] finds the commanded torque for the reaction wheels to be that of equation 23 in the paper:

$$
\begin{align}
    \nonumber \tau_w &= -\omega^\times h_w + T_m - u \\
    &= -\omega^\times h_w - (R(q) \tilde{b}_\circ (t))^\times \tau_m - u \\
\end{align}
$$

where $u$ is the commanded torque provided by the controller to achieve pointing accuracy.
This is implemented as such:

```
hw = self.angular_momentum_wheels()
tau_w = -misc.skew(omega) @ hw + T_m - u
```

`tau_w` is the commanded for the wheels and is distributed in the same fashion as the magnetorquers:

```
self.A = np.array([x.axis for x in self.reaction_wheels]).T
self.A_inv = np.linalg.pinv(self.A)

...

def distribute_torque_rw(self, tau_c: np.ndarray) -> np.ndarray:
    L = np.zeros_like(tau_c)
    u_wheels = self.A_inv @ tau_c    # Torque for each wheel
    for i, ac in enumerate(self.reaction_wheels):
        L += ac.axis * ac.apply_torque(u_wheels[i])
    return L
```

All torques applied from the actuation system is now accounted for, and we can then return the total torque applied:

```
L_w = self.distribute_torque_rw(tau_w)
return -L_w + T_m - misc.skew(omega) @ hw
```

Where $\omega^\times h_w$ is the [gyroscopic precession](https://en.wikipedia.org/wiki/Precession).

## Reference signals

There are currently implemented two methods of generating reference signals. 
The first - called `BaseReference` - is fairly trivial as it just integrates the constant angular velocity that is commanded.
This provides the commanded angular velocity, and its corresponding attitude, to the controller.

Another more complicated method is also implemented called `EarthReference(BaseReference)` (the name is not fantastic).
This method will calculate a reference attitude that points the x-axis in the direction of a provided primary target, and the y-axis in the direction of a provided secondary target.
The reference signal from this method will only calculate the attitude of the reference, and presume the angular velocity to be zero $\omega = 0$.
There is room for improvement of this method by also calculating the angular velocity by interpolating attitudes over some horizon of espected positions for the satellite and targets.
This would have to include a way to propagate the position of the satellite and targets though, which is currently lacking.

The method has the following inputs, all in the same reference frame (presumably ECI or some other inertial frame)

* `sat_pos`: Position of satellite
* `R`: Rotation matrix of satellite body frame with respect to the inertial frame (presumably ECI)
* `target_pos`: Position of the primary target, of which the x-axis of the satellite will point at
* `second_target`: Position of the secondary target, of which the y-axis of the satellite will point at

The x-axis of the rotation matrix will coincide with the vector from the satellite to the primary target as such:
$$
    \vec{x} = \frac{p_{sat} - p_{t1}}{|p_{sat} - p_{t1}|}
$$

The same approach can not be taken for the y-axis however, as it is unlikely to be located such that x-axis and y-axis can point at their targets simultaneously.
With the constraint that the x-axis points in the direction of $\vec{x}$, both the y- and z-axis are constrained to be located on the plane defined by $\vec{x}$.

$$
\begin{align}
    \vec{x} \cdot \vec{y} &= 0 \\
    \vec{x} \cdot \vec{z} &= 0
\end{align}
$$

The direction of the y-axis is calculated by taking the projection of the vector $\vec{s2}$, which is the vector from the satellite position to the secondary target position,
onto the plane $n$ defined by its normal vector $\vec{x}$.

$$
\begin{align}
    \nonumber \vec{s2} &= \frac{p_{sat} - p_{t2}}{|p_{sat} - p_{t2}|} \\
    \nonumber \vec{y} &= \text{proj}_n \vec{s2} \\
    &= \vec{s2} - (\vec{s2} \cdot \vec{x})\vec{x}
\end{align}
$$

The z-axis can then be found by taking the cross product of the x- and y-axis: 
$$
    \vec{z} = \vec{x} \times \vec{y}
$$

Which gives the rotation matrix of the satellite body relative to inertial frame:

$$
\mathbf{R} = 
\begin{bmatrix}
    \vec{x} &  \vec{y} & \vec{z}
\end{bmatrix}
$$

There are some issues that needs to be adressed here however.
If the secondary target is in the same direction as the primary target, i.e. that $\vec{x} = \alpha \vec{s2}$ for some constant $\alpha$, then $\vec{y}$ is the zero vector.
This will cause a crash, so to prevent this, we add a check that will change the secondary target such that the y- and z-axis remain in the current position. 
If the norm of is less than some constant $0 < |\vec{y}| < \beta$ then the y- and z-axis will remain in their current orientation.
This is implemented as such:

```
if (np.linalg.norm(y,2) < 0.1):
    y_sat = R[:, 1]
    z_sat = R[:, 2]
    y = self.project_to_plane(y_sat, x)
    z = self.project_to_plane(z_sat, x)
if np.linalg.norm(y, 2) > np.linalg.norm(z, 2):
    y /= np.linalg.norm(y, 2)
    z = np.cross(x, y)
else:
    z /= np.linalg.norm(z, 2)
    y = np.cross(x, z)
```

The full implementation of the method is in the function `update`:

```
    def update(self, dt: float, **kwargs):
        try:
            # NOTE: ECI Reference frame
            target_pos: np.ndarray = kwargs["target_pos"]
            sat_pos: np.ndarray = kwargs["sat_pos"]
            second_target: np.ndarray = kwargs["second_target"]

            # NOTE: Satellite rotation matrix with respect to inertial frame (ECI)
            sat_rot: Rotation = kwargs["sat_rot"]
            R: np.ndarray = sat_rot.as_matrix() # Store the rotation matrix of sat

        except:
            # Default back to the basic reference if the wrong parameters are provided
            return super().update(dt)

        # Normalized vector from satellite to target position gives the x-axis
        x = target_pos - sat_pos
        x /= np.linalg.norm(x, 2)

        y = self.project_to_plane(second_target, x)

        if (np.linalg.norm(y,2) < 0.1):
            y_sat = R[:, 1]
            z_sat = R[:, 2]
            y = self.project_to_plane(y_sat, x)
            z = self.project_to_plane(z_sat, x)

            if np.linalg.norm(y, 2) > np.linalg.norm(z, 2):
            # NOTE: Normalize before computing the cross product
                y /= np.linalg.norm(y, 2)
                z = np.cross(x, y)
            else:
                z /= np.linalg.norm(z, 2)
                y = np.cross(x, z)
        else:
            y /= np.linalg.norm(y, 2)
            z = np.cross(x, y)

        # NOTE: The reconstruction of the rotation matrix for the reference attitude
        # is handled automatically by scipy.
        self.state.rot = Rotation.from_matrix(np.column_stack((x,y,z)))

        return self.state
```

[^Tregouet_RW]: Jean-François Tregouet, Denis Arzelier, Dimitri Peaucelle, Christelle Pittet, Luca Zaccarian. "Reaction Wheels Desaturation Using Magnetorquers and Static Input Allocation." IEEE Transactions on Control Systems Technology, vol. 23, no. 2, pp. 525-539, 2015. DOI: 10.1109/TCST.2014.2326037.
