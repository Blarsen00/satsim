import yaml
import numpy as np
from Actuators.Actuator import Actuator
import matplotlib.pyplot as plt

def rpm_to_angular_velocity(rpm):
    return rpm * 0.1407

def angular_velocity_to_rpm(rad):
    return rad / 0.1407

class ReactionWheel(Actuator):
    def __init__(self, axis=[1.0, 0.0, 0.0], file="Actuators/Yaml/Reactionwheel.yaml") -> None:
        super().__init__(axis)

        with open(file, 'r') as file:
            self.param = yaml.safe_load(file)
        file.close()

        self.Jw = self.param["inertia_wheel"]
        self.dv = self.param["viscous_damping"]
        self.dc = self.param["coloumb_damping"]
        self.w = self.param["w0"]
        self.max_rpm = self.param["max_rpm"]
        self.max_tau = self.param["max_tau"]

    def drag(self):
        # (4.55)
        return -self.w * self.dv - self.dc * np.sign(self.w)

    def apply_torque(self, L, dt):
        applied_torque = L + self.drag()
        applied_torque = max(min(applied_torque, self.max_tau), -self.max_tau)
        angular_acceleration = applied_torque / self.Jw

        # Models saturation of the wheel rpm
        if angular_acceleration > 0:
            # The amount of acceleration we can have without reaching the max rpm
            max_ang_acc = (rpm_to_angular_velocity(self.max_rpm) - self.w) / dt
            angular_acceleration = min(angular_acceleration, max_ang_acc)
        else:
            #The same, but for negative accelerations
            max_ang_acc = (-rpm_to_angular_velocity(self.max_rpm) - self.w) / dt
            angular_acceleration = max(angular_acceleration, max_ang_acc)

        # Update the angular velocity of the reaction wheel
        self.w += angular_acceleration * dt
        applied_torque = angular_acceleration * self.Jw

        return applied_torque

    def voltage_to_torque(self, v):
        # These values will probably change. Ask Christoffer.
        # implement these values as default parameters in class later?

        G_d = 1
        K_t = 1

        return min(max(G_d * K_t * v, -self.max_tau), self.max_tau)

    def apply_voltage(self, v, dt):
        self.apply_torque(self.voltage_to_torque(v), dt)


def testTorque(rw:ReactionWheel):
    dt = 0.001
    T = np.arange(0.0, 100.0, dt)
    rpm = np.zeros_like(T)
    L = np.zeros_like(T)
    LC = np.zeros_like(T)

    for i, t in enumerate(T):
        rpm[i] = angular_velocity_to_rpm(rw.w)
        LC[i] = 0.5
        # LC[i] = 0.01 * np.sin(t)
        L[i] = rw.apply_torque(LC[i], dt)

    fig_rpm = plt.figure(0)
    plt.title("RPM plot")
    plt.plot(T, rpm)
    # plt.plot([T[0], T[-1]], [rw.max_rpm, rw.max_rpm], 'r--')
    # plt.plot([T[0], T[-1]], [-rw.max_rpm, -rw.max_rpm], 'r--')
    plt.grid()

    fig_L = plt.figure(2)
    plt.title("Torque plot")
    plt.plot(T, L)
    plt.plot(T, LC, '--g')
    plt.plot([T[0], T[-1]], [-rw.max_tau, -rw.max_tau], '--r')
    plt.plot([T[0], T[-1]], [rw.max_tau, rw.max_tau], '--r')
    plt.grid()

    plt.show()


if __name__ == "__main__":
    rw = ReactionWheel()
    testTorque(rw)

