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
        Handy resource: https://www.researchgate.net/publication/263083066_Torque_and_Speed_Control_Loops_of_a_Reaction_Wheel
    """
    name: str = "Reaction Wheel"

    # TODO: Change the variables to those that match the hardware
    # State variables
    # J: np.ndarray = np.eye(3) * 1.53e-3 # Inertia matrix
    # w: float = 0.0                      # Angular velocity

    # TODO: Change the variables to those that match the hardware
    # Damping coefficients
    # dv: float = 4.83e-6                 # Coloumb damping coefficient
    # dc: float = 0.8795e-3               # Viscous damping coefficient

    # TODO: Change the variables to those that match the hardware
    # Motor model
    # km: float = 0.00228
    # max_I: float = 1.0
    # max_rpm: float = 5000.0

    def __init__(self, axis: Optional[np.ndarray] = None) -> None:
        super().__init__(axis)

        # Initialize ALL mutable attributes and parameter defaults here
        # TODO: Find the most reasonable default values
        self.J: np.ndarray = np.eye(3) * 1.53e-3
        self.w: float = 0.0
        self.dv: float = 4.83e-6
        self.dc: float = 0.8795e-3
        self.km: float = 0.00228
        self.max_I: float = 1.0

        # NOTE: Main params are displayed primarily, params are the parameters
        # one can choose if the options are expanded and cover all parameters.
        self._main_params = {"axis": "Axis: ",
                             "max_I": "Max current (A): ",
                             "w": "Angular rate(rad/s)"}
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


    def get_rpm(self, w: Optional[float]=None):
        w = w if w is not None else self.w
        return w * 30 / np.pi

    @staticmethod
    def calculate_max_rpm(max_I: float,
                          km: float,
                          dc: float,
                          dv: float):
        """ Uses the max current for the class to provide to calculate the max possible
            rpm the wheel is able to achieve. That being the angular velocity of which
            the torque the motor is able to provide equals the drag from the wheel.

            tau = I_max km - dv w - dc sign(w)      (Set tau=0, and positive w)
            0 = I_max km - dv w - dc
            w = (I_max km - dc) / dv                (rad/s)
        """
        # max_I = max_I if max_I is not None else cls.max_I
        # km = km if km is not None else cls.km
        #
        # dc = dc if dc is not None else cls.dc
        # dv = dv if dv is not None else cls.dv

        w = (km * max_I - dc) / (dv)
        w = Actuator.saturate(w, minimum=0.0)

        return w * 30 / np.pi

    @classmethod
    def drag(cls, w: float, dc:Optional[ float ]=None, dv:Optional[ float ]=None):
        """ Wheel friction modeled as a sum of viscous and Coloumb components. (4.55)
        """
        dc = dc if dc is not None else cls.dc
        dv = dv if dv is not None else cls.dv

        drag = dv * w + dc * np.sign(w)
        return drag

    @classmethod
    def motor_torque(cls,
                     tau: float,
                     km: Optional[float]=None,
                     max_I: Optional[float]=None,
                     dt: float=0.1) -> Tuple[float, float]:
        """
            Torque provided by the motor to the wheels. Assumed to be proportional 
            to the applied electrical current to the motor.
            T_m = k_m I
        """
        km = km if km is not None else cls.km
        max_I = max_I if max_I is not None else cls.max_I

        # Wheel is saturated at max rpm and cannot spin faster
        I = tau / km
        I = Actuator.saturate(I, max_I, -max_I)

        # Line to force the reaction wheel to saturation
        # I *= int(self.w <= self.max_w)

        # Torque from wheel
        T = km * I

        return T, I

    def wheel_torque(self, tau: float, dt: float):
        """The total torque from the wheel comes from the motor, and the drag.
            Update the angular velocity of the wheel as well.
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
        tau = T * self.axis
        dw = self.J_inv @ tau
        w_diff = dt * np.dot(self.axis, dw)
        self.w += w_diff

        return T

    def apply_torque(self, tau: float, dt: float=0.1):
        T = self.wheel_torque(tau, dt)

        # Log the available data
        self.log_data("time", self.data["time"][-1] + 0.1)
        self.log_data("reference", tau)
        self.log_data("w", self.w)
        self.log_data("rpm", self.get_rpm())
        self.log_data("torque", T)

        return T


##################### Animation part of the reactionwheel ####################
# class ReactionWheelAnimation(ActuatorAnimation):
#     def __init__(self, actuator) -> None:
#         super().__init__(actuator)
#         self.color="blue"


def test_reaction_wheel():
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
    w = np.linspace(0, 4000, 1000)
    d = np.zeros_like(w)
    for i, x in enumerate(w):
        d[i] = ReactionWheel.drag(x)

    ax = plt.subplot()
    ax.plot(w[1:], d[1:])
    ax.axhline(ReactionWheel.motor_torque(10000.0)[0], color="r", linestyle="--")

    plt.tight_layout() # Improves spacing between subplots
    plt.show()


def test_max_rpm():
    I = np.linspace(0, 10.0, 1000)
    rpm = np.zeros_like(I)
    for i, x in enumerate(I):
        rpm[i] = ReactionWheel.calculate_max_rpm(max_I=x)

    ax = plt.subplot()
    ax.plot(I[1:], rpm[1:])

    plt.tight_layout() # Improves spacing between subplots
    plt.show()


def make_reactionwheel_plot(rw: ReactionWheel):
    w = np.zeros(30_000)
    tau = np.zeros(30_000)
    T = np.arange(0, 3000, 0.1)
    for i in range(30_000):
        # tau[i] = np.linalg.norm(wheel.apply_torque(1.01, 0.1), 2)
        tau[i] = rw.apply_torque(1.01, 0.1)
        w[i] = rw.get_rpm()
        # w[i] = wheel.w

    # fig, ax = plt.subplots(2, 1, figsize=(8,6))
    fig, ax = plt.subplots(2, 1)
    ax[0].plot(T[1:], w[1:])
    ax[0].set_title('Angular Velocity vs. Time')
    ax[0].set_ylabel(r'Angular Velocity $\omega$ (rpm)')
    ax[0].axhline(y=rw.max_rpm, color="r", linestyle="--")
    ax[0].grid()

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
    print(f"Max rpm: {ReactionWheel.calculate_max_rpm()}")
    test_max_rpm()
    test_reaction_wheel()

