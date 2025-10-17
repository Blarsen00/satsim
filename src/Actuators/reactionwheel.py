"""
ReactionWheel Module

This module defines the ReactionWheel class, a type of Actuator used for 
attitude control through momentum exchange.
"""
import numpy as np
from Actuators.actuator import Actuator, ActuatorAnimation
import matplotlib.pyplot as plt
from matplotlib.artist import Artist
import matplotlib.patches as mpatch
from typing import Optional, Tuple, Iterable

from animation import BaseAnimation
from collections import deque

class ReactionWheel(Actuator):
    """ 
    Represents a Reaction Wheel (RW) actuator used for spacecraft attitude control.

    The RW generates torque by accelerating or decelerating an internal flywheel, 
    thereby exchanging angular momentum with the spacecraft body.

    Inherits from :class:`Actuators.actuator.Actuator`.

    Attributes
    ----------
    name : str
        The name of the actuator: "Reaction Wheel".
    J : :class:`numpy.ndarray`
        The inertia tensor of the reaction wheel (typically a diagonal 3x3 matrix) 
        expressed in the wheel's rotation frame. Defaults to :math:`1.53 \\times 10^{-3} \\text{ kg} \\cdot \\text{m}^2`.
    w : float
        The current angular velocity of the reaction wheel about its axis (rad/s).
    dv : float
        Viscous damping coefficient.
    dc : float
        Coulomb damping coefficient.
    km : float
        Motor torque constant, relating current :math:`I` to motor torque :math:`\\tau_m`: 
        :math:`\\tau_m = k_m I`.
    max_I : float
        Maximum available electrical current (A) for the motor.
    J_inv : :class:`numpy.ndarray`
        The inverse of the inertia tensor :math:`\\mathbf{J}`.
    max_rpm : float
        The maximum attainable rotational speed (RPM) based on motor limits and drag.
    max_w : float
        The maximum attainable angular velocity (rad/s).
    """
    # Handy resource: 
    # https://www.researchgate.net/publication/263083066_Torque_and_Speed_Control_Loops_of_a_Reaction_Wheel
    name: str = "Reaction Wheel"

    def __init__(self, axis: Optional[np.ndarray] = None) -> None:
        """
        Initializes the ReactionWheel instance with default parameters.

        Parameters
        ----------
        axis : :class:`numpy.ndarray`, optional
            The direction vector of the reaction wheel's axis in the body frame. 
            Inherited from :class:`Actuators.actuator.Actuator`.
        """
        super().__init__(axis)

        # TODO: Find the most reasonable default values
        self.J: np.ndarray = np.eye(3) * 1.53e-3
        self.w: float = 0.0                      # rad/s
        # NOTE: The original comments incorrectly swapped the definitions of dv and dc.
        # dv is Viscous (proportional to w), dc is Coulomb (proportional to sign(w)).
        self.dv: float = 4.83e-6                  # Viscous damping coefficient
        self.dc: float = 0.8795e-3                # Coulomb damping coefficient
        self.km: float = 0.00228                  # Relation between current and torque
        self.max_I: float = 1.0

        # NOTE: Main params are displayed primarily, params are the parameters
        # one can choose if the options are expanded and cover all parameters.
        # self._main_params = {"axis": "Axis: ",
        #                         "max_I": "Max current (A): ",
        #                         "w": "Angular rate(rad/s)"}
        self._params = ["axis", "w", "max_I", "km", "dv", "dc", "J"]

        # Store inverse so it isn't necessary to calculate it at each iteration
        self.J_inv = np.linalg.inv(self.J)
        self.max_rpm = self.calculate_max_rpm(self.max_I,
                                              self.km,
                                              self.dc,
                                              self.dv)
        self.max_w = self.max_rpm * np.pi / 30

        # Just add some dummy data at the start
        self.log_data("torque", 0)
        self.log_data("reference", 0)
        self.log_data("I", 0)
        self.log_data("motor", 0)
        self.log_data("drag", 0)
        self.log_data("w", 0)
        self.log_data("rpm", 0)
        # self.log_data("w_dot", 0)


    def get_rpm(self, w: Optional[float]=None) -> float:
        """ 
        Converts angular velocity from rad/s to Revolutions Per Minute (RPM). 
        If no angular velocity is provided, the wheel's current angular rate is used.

        Parameters
        ----------
        w : float, optional
            The angular velocity in rad/s. If None, uses `self.w`.

        Returns
        -------
        float
            The angular velocity in RPM.
        """
        w = w if w is not None else self.w
        return w * 30 / np.pi

    @staticmethod
    def calculate_max_rpm(max_I: float,
                          km: float,
                          dc: float,
                          dv: float) -> float:
        """ 
        Calculates the maximum possible RPM the wheel is able to achieve. 
        This is the angular velocity at which the motor's maximum torque 
        equals the total drag torque.

        The formula derived from :math:`\\tau_{\\text{motor}} = I_{\\text{max}} k_m` 
        and the drag torque :math:`\\tau_{\\text{drag}} = d_v \\omega + d_c \\text{sgn}(\\omega)` 
        by setting :math:`\\tau_{\\text{motor}} = \\tau_{\\text{drag}}` and assuming positive :math:`\\omega` is:
        
        .. math::
            \\omega_{\\text{max}} = \\frac{k_m I_{\\text{max}} - d_c}{d_v} \\quad (\\text{rad/s})
        

        Parameters
        ----------
        max_I : float
            Maximum available electrical current (A).
        km : float
            Motor torque constant (:math:`k_m`).
        dc : float
            Coulomb damping coefficient (:math:`d_c`).
        dv : float
            Viscous damping coefficient (:math:`d_v`).

        Returns
        -------
        float
            The maximum angular velocity in RPM.
        """

        w = (km * max_I - dc) / (dv)
        w = Actuator.saturate(w, minimum=0.0)    # I cannot remember why i saturate here, but I'm sure it's well reasoned

        # Convert from rad/s to rpm
        return w * 30 / np.pi

    @staticmethod
    def drag(w: float, dc:float, dv: float) -> float:
        """ 
        Calculates the friction torque (drag) acting on the wheel, modeled as a 
        sum of viscous and Coulomb components:
        
        .. math::
            \\tau_{\\text{drag}} = d_v \\omega + d_c \\text{sgn}(\\omega)
        

        This model may not perform well near zero velocity.

        Parameters
        ----------
        w : float
            The current angular velocity of the wheel (rad/s).
        dc : float
            Coulomb damping coefficient (:math:`d_c`).
        dv : float
            Viscous damping coefficient (:math:`d_v`).

        Returns
        -------
        float
            The total drag torque (:math:`\\tau_{\\text{drag}}`) (Nm).
        """

        drag = dv * w + dc * np.sign(w)
        return drag

    def motor_torque(self,
                     tau: float,
                     km: Optional[float]=None,
                     max_I: Optional[float]=None,
                     dt: float=0.1) -> Tuple[float, float]:
        """
        Calculates the torque provided by the motor to the wheels, assumed to be 
        proportional to the applied electrical current and saturated within the 
        motor's current limits.

        .. math::
            \\tau_m = k_m I
        

        Parameters
        ----------
        tau : float
            The *requested* motor torque (Nm).
        km : float, optional
            Motor torque constant (:math:`k_m`). If None, uses `self.km`.
        max_I : float, optional
            Maximum available current (A). If None, uses `self.max_I`.
        dt : float, optional
            Time step duration (s). Not used in this calculation.

        Returns
        -------
        Tuple[float, float]
            A tuple containing: 
            1. The actual motor torque (:math:`\\tau_m`) (Nm).
            2. The resulting current (:math:`I`) (A).
        """
        km = km if km is not None else self.km
        max_I = max_I if max_I is not None else self.max_I

        # Saturate the current within the capabilities of the motor
        I = tau / km
        I = Actuator.saturate(I, max_I, -max_I)

        # Line to force the reaction wheel to saturation, currently depreciated
        # I *= int(self.w <= self.max_w)

        # Torque from wheel
        T = km * I

        return T, I

    def wheel_torque(self, tau: float, dt: float) -> float:
        """
        Calculates the total torque acting on the wheel (:math:`\\tau_{\\text{motor}} - \\tau_{\\text{drag}}`)
        and updates the wheel's angular velocity (:math:`\\omega`).

        The motor is commanded to overcome the requested torque :math:`\\tau` plus the internal drag: 
        :math:`\\tau_{\\text{motor}} = \\tau_{\\text{commanded}} + \\tau_{\\text{drag}}`.

        Parameters
        ----------
        tau : float
            The commanded torque magnitude (Nm) from the control system.
        dt : float
            The time step duration (s).

        Returns
        -------
        float
            The total resulting torque (Nm) applied by the wheel on the spacecraft body.
        """
        drag = self.drag(self.w, self.dc, self.dv)

        # Motor torque needs to account for the induced drag from the rotation
        # This gives the reference for the motor: tau_m = tau + tau_d
        motor, I = self.motor_torque(tau + drag, self.km, self.max_I, dt)

        self.log_data("I", I)
        self.log_data("motor", motor)
        self.log_data("drag", drag)

        T = motor - drag

        # Update the state of the wheel
        # The equation of motion for the wheel is J * dw/dt = T_motor - T_drag
        # T_net = T_motor - T_drag = T
        # dw/dt = J_inv * T_net_vector
        # The wheel rotates only about its axis, so the net torque vector is T * axis.
        # The change in angular velocity vector is dw = J_inv @ (T * axis).
        # We only care about the magnitude of the angular velocity change along the axis.
        tau_vec = T * self.axis
        dw = self.J_inv @ tau_vec
        # The change in the scalar angular speed w is the component of dw along the axis:
        w_diff = dt * np.dot(self.axis, dw) 
        self.w += w_diff

        # The torque applied by the wheel on the body is $-\mathbf{\tau}_{\text{net}}$ along the axis.
        # Since T is the scalar magnitude of $\mathbf{\tau}_{\text{net}}$, the scalar magnitude of 
        # the reaction torque $\mathbf{\tau}_{\text{reaction}} = -\mathbf{\tau}_{\text{net}}$ is also T.
        # This function should return the magnitude of the torque applied to the body.
        return -T # $\tau_{\text{body}} = -\tau_{\text{wheel}}$

    def apply_torque(self, tau: float, dt: float=0.1, **kwargs) -> float:
        """
        The main interface method for the actuator system. It commands the reaction 
        wheel to produce a torque magnitude :math:`\\tau`.

        This method updates the internal state of the reaction wheel (angular velocity) 
        and calculates the actual torque produced.

        Parameters
        ----------
        tau : float
            The scalar magnitude of the commanded torque (Nm).
        dt : float, optional
            The time step duration (s). Defaults to 0.1 s.
        **kwargs : dict
            Additional parameters (currently unused).

        Returns
        -------
        float
            The scalar magnitude of the resulting torque (Nm) applied to the body.
        """
        T = self.wheel_torque(tau, dt)

        # Log the available data
        # NOTE: The original code has a bug: it increments time by a fixed 0.1 and not dt, 
        # and assumes the last entry of "time" exists. 
        # I'll keep the original time logging logic for now, but note it might be incorrect 
        # if the calling system uses variable dt or doesn't initialize 'time' properly.
        # For a clean module, the log_data in wheel_torque should be sufficient.
        
        # self.log_data("time", self.data["time"][-1] + 0.1) # Time logging is better handled externally
        self.log_data("reference", tau)
        self.log_data("w", self.w)
        self.log_data("rpm", self.get_rpm())
        self.log_data("torque", T)

        return T


def test_reaction_wheel():
    """
    Simulates a reaction wheel under a constant commanded torque (1.01 Nm) over 
    a period of 3000 seconds to observe angular velocity and saturation.
    """
    wheel = ReactionWheel()
    w = np.zeros(30_000)
    tau = np.zeros(30_000)
    T = np.arange(0, 3000, 0.1)
    for i in range(30_000):
        # tau[i] = np.linalg.norm(wheel.apply_torque(1.01, 0.1), 2)
        tau[i] = wheel.apply_torque(1.01, 0.1)
        w[i] = wheel.get_rpm()
        # w[i] = wheel.w

    fig, ax = plt.subplots(2, 1, figsize=(8,6))
    ax[0].plot(T[1:], w[1:])
    ax[0].set_title('Angular Velocity vs. Time')
    ax[0].set_ylabel('Angular Velocity (rpm)')
    ax[0].axhline(y=wheel.max_rpm, color="r", linestyle="--")

    ax[1].plot(T[1:], tau[1:])
    ax[1].set_title('Applied Torque vs. Time')
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel('Torque (Nm)')

    plt.tight_layout() # Improves spacing between subplots
    plt.show()


def test_drag():
    """
    Plots the drag torque as a function of angular velocity (rad/s) to visualize 
    the viscous and Coulomb friction model.
    """
    rw = ReactionWheel() # Instantiate to get drag coefficients
    w = np.linspace(0, 4000, 1000)
    d = np.zeros_like(w)
    for i, x in enumerate(w):
        d[i] = rw.drag(x, rw.dc, rw.dv) # Pass coefficients explicitly

    ax = plt.subplot()
    ax.plot(w[1:], d[1:])
    # NOTE: The motor_torque call here is meaningless as it returns a tuple (T, I) and needs tau.
    # The original intention was likely to show the max motor torque line.
    # max_motor_torque = rw.km * rw.max_I 
    # ax.axhline(max_motor_torque, color="r", linestyle="--") 
    ax.set_title('Drag Torque vs. Angular Velocity')
    ax.set_xlabel('Angular Velocity $\omega$ (rad/s)')
    ax.set_ylabel('Drag Torque $\\tau_{\\text{drag}}$ (Nm)')
    ax.grid(True)
    
    plt.tight_layout() # Improves spacing between subplots
    plt.show()


def test_max_rpm():
    """
    Plots the calculated maximum RPM as a function of varying maximum current (:math:`I_{\\text{max}}`) 
    to demonstrate the relationship between power limits and speed capacity.
    """
    rw = ReactionWheel() # Instantiate to get constants
    I = np.linspace(0, 10.0, 1000)
    rpm = np.zeros_like(I)
    for i, x in enumerate(I):
        # Pass constants explicitly, as calculate_max_rpm is static
        rpm[i] = ReactionWheel.calculate_max_rpm(max_I=x, km=rw.km, dc=rw.dc, dv=rw.dv)

    ax = plt.subplot()
    ax.plot(I[1:], rpm[1:])
    ax.set_title('Max RPM vs. Max Current')
    ax.set_xlabel('Maximum Current $I_{\\text{max}}$ (A)')
    ax.set_ylabel('Maximum Angular Velocity $\\omega_{\\text{max}}$ (RPM)')
    ax.grid(True)

    plt.tight_layout() # Improves spacing between subplots
    plt.show()


def make_reactionwheel_plot(rw: ReactionWheel):
    """
    Executes a simulation run on a ReactionWheel instance and generates a two-panel 
    matplotlib plot showing the angular velocity (RPM) and applied torque (Nm) over time.

    Parameters
    ----------
    rw : :class:`ReactionWheel`
        The ReactionWheel instance to simulate and plot.

    Returns
    -------
    :class:`matplotlib.figure.Figure`
        The generated matplotlib figure object.
    """
    w = np.zeros(30_000)
    tau = np.zeros(30_000)
    T = np.arange(0, 3000, 0.1)
    
    # Re-initialize the wheel state for a clean run
    rw.w = 0.0
    
    for i in range(30_000):
        # tau[i] stores the scalar magnitude of the torque applied to the body
        tau[i] = rw.apply_torque(1.01, 0.1)
        w[i] = rw.get_rpm()

    # fig, ax = plt.subplots(2, 1, figsize=(8,6))
    fig, ax = plt.subplots(2, 1)
    ax[0].plot(T[1:], w[1:])
    ax[0].set_title('Angular Velocity vs. Time')
    ax[0].set_ylabel(r'Angular Velocity $\omega$ (rpm)')
    ax[0].axhline(y=rw.max_rpm, color="r", linestyle="--", label='Max RPM Limit')
    ax[0].grid()
    ax[0].legend()

    # Create a patch and text for max RPM
    max_rpm = np.max(w)
    ax[0].text(0.95, 0.05, f'Max RPM: {max_rpm:.2f}',
                 horizontalalignment='right', verticalalignment='bottom',
                 transform=ax[0].transAxes,
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='lightblue', alpha=0.5))

    ax[1].plot(T[1:], tau[1:])
    ax[1].set_title('Applied Torque vs. Time')
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel(r'Torque $\tau$ (Nm)')
    ax[1].grid()

    # Create a patch and text for max Torque
    max_tau = np.max(tau)
    ax[1].text(0.95, 0.95, f'Max Torque: {max_tau:.6f} Nm',
                 horizontalalignment='right', verticalalignment='top',
                 transform=ax[1].transAxes,
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.5))

    plt.tight_layout() # Improves spacing between subplots
    return fig


if __name__ == "__main__":
    test_max_rpm()
    test_reaction_wheel()
    # test_drag() # Included for completeness, though commented out in original __main__
