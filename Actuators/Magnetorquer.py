import yaml
import numpy as np
import matplotlib.pyplot as plt
from Actuators.Actuator import Actuator


class Magnetorquer(Actuator):
    def __init__(self, axis=[1.0, 0.0, 0.0], file="Actuators/Yaml/Magnettorquer.yaml") -> None:
        super().__init__(axis)

        with open(file, 'r') as file:
            self.param = yaml.safe_load(file)
        file.close()

        self.scaling = self.param["scalingFactor"]
        self.maxCurrent = self.param["maxCurrent"]
        self.maxTorque = self.param["maxTorque"]


    def apply_torque(self, L, dt):
        # Assume instantanious torque, so dt becomes irrelevant
        tau = min(abs(L), self.maxTorque)
        tau *= np.sign(L)
        return tau

def testTorque(mqt:Magnetorquer):
    dt = 0.001
    T = np.arange(0.0, 100.0, dt)
    L = np.zeros_like(T)
    LC = np.zeros_like(T)

    for i, t in enumerate(T):
        # LC[i] = 0.6 * mqt.maxTorque
        LC[i] = 1.2 * mqt.maxTorque * np.sin(t)
        L[i] = mqt.apply_torque(LC[i], dt)

    fig_L = plt.figure(2)
    plt.title("Torque plot")
    plt.plot(T, L, 'b')
    plt.plot(T, LC, '--g')
    plt.plot([T[0], T[-1]], [-mqt.maxTorque, -mqt.maxTorque], '--r')
    plt.plot([T[0], T[-1]], [mqt.maxTorque, mqt.maxTorque], '--r')
    plt.grid()

    plt.show()

if __name__ == "__main__":
    mtq = Magnetorquer()
    testTorque(mtq)

